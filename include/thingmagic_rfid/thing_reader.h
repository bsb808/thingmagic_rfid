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


#ifndef _THING_READER_H
#define _THING_READER_H

#include <cstdio>
#include <unistd.h>

extern "C"{
#include "tm_reader.h"
}
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <inttypes.h>
#include <unistd.h>


// ROS
#include "ros/ros.h"
#include "std_msgs/String.h"

/**
 * \brief Contains functions for micostrain driver
 */
namespace TMR
{
  /**
   * \brief TMR class
   * 
   */
  class ThingReader
  {
  public:
    /**
     * Contructor 
     */
    ThingReader();

    /** Destructor */
    ~ThingReader();

    /** 
     * Main run loop
     */
    int run();

    void rfid_callback(TMR_Reader *reader, const TMR_TagReadData *t, void *cookie);

  private:
    ros::Publisher string_pub_;
    std_msgs::String string_msg_;


 
  }; // TMR class

  void errx(int exitval, const char *fmt, ...);
  void checkerr(TMR_Reader* rp, TMR_Status ret, int exitval, const char *msg);
  void callback_wrapper(TMR_Reader *reader, const TMR_TagReadData *t, void *cookie);
  void exceptionCallback(TMR_Reader *reader, TMR_Status error, void *cookie);

} // namespace TMR






#endif 
