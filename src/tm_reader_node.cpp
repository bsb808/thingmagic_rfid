/* 

Copyright (c) 2017, Brian Bingham
All rights reserved

This file is part of the thinkmagic_rfid package.

The package is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

The package is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with the package.  If not, see <http://www.gnu.org/licenses/>.

*/

/**
 * @file    tm_reader_node.cpp
 * @author  Brian S. Bingham
 * @version 0.1
 *
 * @brief ROS Node for Thinkmagic RFID reader
 *
 */


#include "tm_reader.h"
#include <ros/ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tm_reader_node");
  TMR::TMR tm_reader;
  tm_reader.run();
  return 0;
}
