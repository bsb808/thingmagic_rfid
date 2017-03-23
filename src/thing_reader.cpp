/* 
Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the thingmagic_rfid package.

thingmagic_rfid is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

thingmagic_rfid is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with the package.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "thing_reader.h"
#include <string>


/* Enable this to use transportListener 
Not supported here!!!
*/
				    
#ifndef USE_TRANSPORT_LISTENER
#define USE_TRANSPORT_LISTENER 0
#endif

#define usage() {errx(1, "Please provide reader URL, such as:\n"\
                         "tmr:///com4 or tmr:///com4 --ant 1,2\n"\
                         "tmr://my-reader.example.com or tmr://my-reader.example.com --ant 1,2\n");}

namespace TMR
{
  ThingReader::ThingReader()
  {
   

  }
  ThingReader::~ThingReader()
  {
    // pass
  }
  int ThingReader::run()
  {
    
    // ROS setup
    ros::Time::init();
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    ROS_INFO("Thingmagic Reader");

    // ROS Parameters
    // Comms Parameters
    std::string port;
    private_nh.param("port", port, std::string("/dev/ttyACM0"));
    std::string uri = "tmr://"+port;
    // ROS publisher
    this->string_pub_ = node.advertise<std_msgs::String>("rfid",100);

    // Setup TMR reader - taken from readasync.c example in SDK
#ifndef TMR_ENABLE_BACKGROUND_READS
    errx(1, "This sample requires background read functionality.\n"
	 "Please enable TMR_ENABLE_BACKGROUND_READS in tm_config.h\n"
	 "to run this codelet\n");
    return -1;
#else
    TMR_Reader r, *rp;
    TMR_Status ret;
    TMR_Region region;
    TMR_ReadPlan plan;
    TMR_ReadListenerBlock rlb;
    TMR_ReadExceptionListenerBlock reb;
    uint8_t *antennaList = NULL;
    uint8_t buffer[20];
    uint8_t i;
    uint8_t antennaCount = 0x0;
    TMR_String model;
    char str[64];

    rp = &r;
    //ret = TMR_create(rp, argv[1]);
    ROS_INFO("Creating reader object with URI <%s>",uri.c_str());
    ret = TMR_create(rp,uri.c_str());
    checkerr(rp, ret, 1, "creating reader");


    ret = TMR_connect(rp);
    checkerr(rp, ret, 1, "connecting reader");

    region = TMR_REGION_NONE;
    ret = TMR_paramGet(rp, TMR_PARAM_REGION_ID, &region);
    checkerr(rp, ret, 1, "getting region");

    if (TMR_REGION_NONE == region){
      TMR_RegionList regions;
      TMR_Region _regionStore[32];
      regions.list = _regionStore;
      regions.max = sizeof(_regionStore)/sizeof(_regionStore[0]);
      regions.len = 0;
      
      ret = TMR_paramGet(rp, TMR_PARAM_REGION_SUPPORTEDREGIONS, &regions);
      checkerr(rp, ret, __LINE__, "getting supported regions");
      
      if (regions.len < 1){
	checkerr(rp, TMR_ERROR_INVALID_REGION, __LINE__, "Reader doesn't supportany regions");
      }
      region = regions.list[0];
      ret = TMR_paramSet(rp, TMR_PARAM_REGION_ID, &region);
      checkerr(rp, ret, 1, "setting region");  
    }
    
  model.value = str;
  model.max = 64;
  TMR_paramGet(rp, TMR_PARAM_VERSION_MODEL, &model);
  if (((0 == strcmp("Sargas", model.value)) || (0 == strcmp("M6e Micro", model.value)) ||(0 == strcmp("M6e Nano", model.value)))
    && (NULL == antennaList))
  {
    ROS_ERROR("Module doesn't has antenna detection support please provide antenna list\n");
    usage();
  }

  /**
  * for antenna configuration we need two parameters
  * 1. antennaCount : specifies the no of antennas should
  *    be included in the read plan, out of the provided antenna list.
  * 2. antennaList  : specifies  a list of antennas for the read plan.
  **/ 

  // initialize the read plan 
  ret = TMR_RP_init_simple(&plan, antennaCount, antennaList, TMR_TAG_PROTOCOL_GEN2, 1000);
  checkerr(rp, ret, 1, "initializing the  read plan");

  /* Commit read plan */
  ret = TMR_paramSet(rp, TMR_PARAM_READ_PLAN, &plan);
  checkerr(rp, ret, 1, "setting read plan");

  rlb.listener = callback_wrapper;
  rlb.cookie = this;

  reb.listener = exceptionCallback;
  reb.cookie = NULL;

  ret = TMR_addReadListener(rp, &rlb);
  checkerr(rp, ret, 1, "adding read listener");

  ret = TMR_addReadExceptionListener(rp, &reb);
  checkerr(rp, ret, 1, "adding exception listener");

  ret = TMR_startReading(rp);
  checkerr(rp, ret, 1, "starting reading");

  //usleep(5000000);
  ros::spin();

  ret = TMR_stopReading(rp);
  checkerr(rp, ret, 1, "stopping reading");

  TMR_destroy(rp);
  return 0;

#endif /* TMR_ENABLE_BACKGROUND_READS */


  } // end of ThingReader::run()

  void errx(int exitval, const char *fmt, ...)
  {
    va_list ap;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    ROS_FATAL(fmt,ap);
    exit(exitval);
  }

  void checkerr(TMR_Reader* rp, TMR_Status ret, int exitval, const char *msg)
  {
    if (TMR_SUCCESS != ret)
      {
	errx(exitval, "Error %s: %s\n", msg, TMR_strerr(rp, ret));
      }
  }


  void callback_wrapper(TMR_Reader *reader, const TMR_TagReadData *t, void *cookie)
  {
    ThingReader* treader = (ThingReader*) cookie;
    treader->rfid_callback(reader,t,cookie);
  }
  void ThingReader::rfid_callback(TMR_Reader *reader, const TMR_TagReadData *t, void *cookie)
  {
    char epcStr[128];
    TMR_bytesToHex(t->tag.epc, t->tag.epcByteCount, epcStr);
    ROS_DEBUG("Background read: %s\n", epcStr);
    string_msg_.data = std::string(epcStr);
    string_pub_.publish(string_msg_);
  }


  void exceptionCallback(TMR_Reader *reader, TMR_Status error, void *cookie)
  {
    ROS_ERROR("Error:%s\n", TMR_strerr(reader, error));
  }


} // TMR namespace

			  /*


			  */
