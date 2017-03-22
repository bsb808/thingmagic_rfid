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

#include "tm_reader.h"
#include <string>

namespace TMR
{
  TMR::TMR()
  {
    // pass
  }
  TMR::~TMR()
  {
    // pass
  }
  void TMR::run()
  {
    // ROS setup
    ros::Time::init();
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    // ROS Parameters
    // Comms Parameters
    std::string port;
    int baud;
    private_nh.param("port", port, std::string("/dev/ttyACM0"));
    private_nh.param("baudrate",baud,115200);
  } // end of TMR::run()

} // TMR namespace

