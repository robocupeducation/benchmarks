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
#ifndef restaurant_EXECUTOR_H_
#define restaurant_EXECUTOR_H_

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/Empty.h"
#include <tf2/LinearMath/Quaternion.h>
#include "restaurant_HFSM.h"
#include <string>
#include <map>

class restaurant_executor : public bica::restaurant_HFSM
{
public:
  //constructor
  restaurant_executor();
  void init_knowledge();
  //void understand_object_code_iterative();
	//void understand_object_code_once();
	//void navigate_to_end_code_iterative();
	//void navigate_to_end_code_once();
	//void Init_code_iterative();
	void Init_code_once();
	//void understand_location_code_iterative();
	//void understand_location_code_once();
	void navigate_to_init_code_iterative();
	void navigate_to_init_code_once();
	//void navigate_to_location_code_iterative();
	//void navigate_to_location_code_once();
	//void aproach_object_code_iterative();
	//void aproach_object_code_once();
	void aproach_person_code_iterative();
	void aproach_person_code_once();

  bool navigate_to_init_2_aproach_person();
	//bool navigate_to_location_2_aproach_object();
	//bool understand_object_2_understand_location();
	//bool aproach_person_2_understand_object();
	//bool aproach_object_2_navigate_to_end();
	bool Init_2_navigate_to_init();
	//bool understand_location_2_navigate_to_location();


  void talk(std::string str);
  void addMapElement(float px, float py, float pz, float orientation, std::string key);
  void targetReachedCb(const std_msgs::Empty::ConstPtr& msg);

private:
  ros::NodeHandle nh_;
  ros::Publisher talk_pub, navigate_pub;
  ros::Subscriber loc_reached_sub;
  std::map<std::string, geometry_msgs::PoseStamped> locations_map;
  bool loc_reached;
};

#endif
