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


#ifndef _TM_READER_H
#define _TM_READER_H

#include <cstdio>
#include <unistd.h>


// ROS
#include "ros/ros.h"

/**
 * \brief Contains functions for micostrain driver
 */
namespace TMR
{
  /**
   * \brief TMR class
   * 
   */
  class TMR
  {
  public:
    /**
     * Contructor 
     */
    TMR();

    /** Destructor */
    ~TMR();

    /** 
     * Main run loop
     */
    void run();
   
  }; // TMR class

  
} // namespace TMR

#endif 