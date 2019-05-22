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
/* Mantainer: Jonatan Gines jonatan.gines@urjc.es */

#include "restaurant_executor.h"

restaurant_executor::restaurant_executor()
{
  loc_reached = 0;
  navigate_pub = nh_.advertise<geometry_msgs::PoseStamped>("/navigate_to", 1);
  loc_reached_sub = nh_.subscribe("/navigate_to/goal_reached", 1, &restaurant_executor::targetReachedCb, this);

  init_knowledge();
}

void restaurant_executor::init_knowledge()
{
  // definir un geometry_msgs::PoseStamped como punto init
  addMapElement(0.73, 2.35, 0.0, -3.1416 / 2.0, "init");
  addMapElement(0.95, -1.44, 0.0, -3.1416, "kitchen");
  addMapElement(3.11, 1.33, 0.0, 3.1416 / 2.0, "bedroom");
}

void restaurant_executor::Init_code_once()
{
  ROS_WARN("State Init");
  //Say that Restaurant starts
  std::string str = "restaurant Starts";
  talk(str);
}

void restaurant_executor::navigate_to_init_code_iterative()
{
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
  it = locations_map.find("init");
  ROS_INFO("%f %f", it->second.pose.position.x, it->second.pose.position.y);
  navigate_pub.publish(it->second);
}

void restaurant_executor::navigate_to_init_code_once()
{
  ROS_WARN("State Navigate_to_init");
}

void restaurant_executor::aproach_person_code_iterative()
{

}
void restaurant_executor::aproach_person_code_once()
{
  ROS_WARN("State Aproach_Person");
  addDependency("Person_Followed_Publisher");
  addDependency("PD_Algorithm");
}

bool restaurant_executor::Init_2_navigate_to_init()
{
  return true;
}

bool restaurant_executor::navigate_to_init_2_aproach_person()
{
  return loc_reached;
}


void restaurant_executor::talk(std::string str)
{
  std_msgs::String msg;
  msg.data = str;
  talk_pub.publish(msg);
}

void restaurant_executor::targetReachedCb(const std_msgs::Empty::ConstPtr& msg)
{
  loc_reached = 1;
}

void restaurant_executor::addMapElement(float px, float py, float pz, float orientation, std::string key)
{
  geometry_msgs::PoseStamped loc;
  loc.pose.position.x = px;
  loc.pose.position.y = py;
  loc.pose.position.z = pz;

  tf2::Quaternion q;
  q.setRPY(0, 0, orientation);
  loc.pose.orientation.x = q[0];
  loc.pose.orientation.y = q[1];
  loc.pose.orientation.z = q[2];
  loc.pose.orientation.w = q[3];

  locations_map[key] = loc;
}
