/*********************************************************************
*  Software License Agreement (BSD License)
*
*   Copyright (c) 2018, Intelligent Robotics
*   All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions
*   are met:

*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of Intelligent Robotics nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.

*   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */
#include "HelpMeCarry_executor.h"

#include <string>

HelpMeCarry_executor::HelpMeCarry_executor()
{
  init_knowledge();
}


void HelpMeCarry_executor::init_knowledge()
{

}

void HelpMeCarry_executor::Init_code_once()
{
  //addDependency("sound_detector");
  ROS_INFO("State Init_code_iterative");
}

void HelpMeCarry_executor::Init_code_iterative()
{

}

void HelpMeCarry_executor::navigate_to_init_code_once()
{
  // publicar en /navigate_to un mensaje de tipo geometry_msgs::PoseStamped con la posicion y orientacion del robot
  // recibiremos un mensaje en /navigate_to/goal_reached std_msgs::Empty cuando el robot llegue a su destino
}

void HelpMeCarry_executor::navigate_to_init_code_iterative()
{
  ROS_INFO("State navigate_to_init_code_iterative");
}


bool HelpMeCarry_executor::Init_2_navigate_to_init()
{
  return true;
}
