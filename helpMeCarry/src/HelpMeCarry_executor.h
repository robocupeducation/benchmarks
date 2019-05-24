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

/* Author: Jonatan Gines jonatan.gines@urjc.es */

/* Author: Jonatan Gines jonatan.gines@urjc.es */
#ifndef HELPMECARRY_EXECUTOR_H_
#define HELPMECARRY_EXECUTOR_H_

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Empty.h"
#include <tf2/LinearMath/Quaternion.h>
#include "HelpMeCarry_HFSM.h"
#include <string>
#include <map>

class HelpMeCarry_executor : public bica::HelpMeCarry_HFSM
{
public:
  //constructor
  HelpMeCarry_executor();

  //bool update();
  void init_knowledge();
  void navigate_to_loc_code_iterative();
	void navigate_to_loc_code_once();
	//void follow_person_code_iterative();
	void follow_person_code_once();
	//void Init_code_iterative();
	void Init_code_once();
	void understanding_next_location_code_iterative();
	void understanding_next_location_code_once();
	void navigate_to_init_code_iterative();
	void navigate_to_init_code_once();

  //void End_code_iterative();
	void End_code_once();
  void searching_person_code_iterative();
	void searching_person_code_once();

	bool navigate_to_loc_2_End();
  bool understanding_next_location_2_navigate_to_loc();
  bool navigate_to_init_2_searching_person();
	bool searching_person_2_follow_person();
	bool Init_2_navigate_to_init();
	bool follow_person_2_understanding_next_location();

  void targetReachedCb(const std_msgs::Empty::ConstPtr& msg);
  void nextLocationCb(const std_msgs::String::ConstPtr& msg);
  void orderCb(const std_msgs::String::ConstPtr& msg);
  void addMapElement(float px, float py, float pz, float orientation, std::string key);
  void errorCb(const std_msgs::String::ConstPtr& msg);
  void talk(std::string str);

private:
  ros::NodeHandle nh_;
  ros::Publisher talk_pub, navigate_pub;
  ros::Subscriber loc_reached_sub, next_location_sub, orders_sub, errorsSub;
  std::map<std::string, geometry_msgs::PoseStamped> locations_map;
  std::string nextLocation;
  int i;
  bool loc_reached, understanding_loc2go_loc, follow_person2understanding_location;
};

#endif
