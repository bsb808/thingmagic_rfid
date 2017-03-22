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

namespace TMR
{
  ThingReader::ThingReader()
  {
    // pass
  }
  ThingReader::~ThingReader()
  {
    // pass
  }
  void ThingReader::run()
  {
    
    // ROS setup
    ros::Time::init();
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    ROS_INFO("Thingmagic Reader");

    // ROS Parameters
    // Comms Parameters
    std::string port;
    int baud;
    private_nh.param("port", port, std::string("/dev/ttyACM0"));
    private_nh.param("baudrate",baud,115200);

    // Setup TMR reader - taken from readasync.c example in SDK

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
    ret = TMR_create(rp,"tmr:///dev/ttyACM0");
    //checkerr(rp, ret, 1, "creating reader");


  } // end of ThingReader::run()


} // ThingReader namespace

			  /*
void errx(int exitval, const char *fmt, ...)
{
  va_list ap;
  
  va_start(ap, fmt);
  vfprintf(stderr, fmt, ap);
  
  exit(exitval);
}

void checkerr(TMR_Reader* rp, TMR_Status ret, int exitval, const char *msg)
{
  if (TMR_SUCCESS != ret)
    {
      errx(exitval, "Error %s: %s\n", msg, TMR_strerr(rp, ret));
    }
}
			  */
