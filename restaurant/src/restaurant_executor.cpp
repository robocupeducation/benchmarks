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
  loc_reached = false;
  maxDist = 1200.0;
  personSaw = false;
  objRecogFinished = false;
  objArrived = false;
  understanding_loc2go_loc = false;
  objUnderstood = false;
  talk_pub = nh_.advertise<std_msgs::String>("/talk", 1);
  navigate_pub = nh_.advertise<geometry_msgs::PoseStamped>("/navigate_to", 1);
  loc_reached_sub = nh_.subscribe("/navigate_to/goal_reached", 1, &restaurant_executor::targetReachedCb, this);
  personDataSub = nh_.subscribe("/person_followed_data", 1, &restaurant_executor::personDataCb, this);
  //objectSub = nh_.subscribe("/object_detected", 1, &restaurant_executor::objectCb, this);
  objRecogFinishedSub = nh_.subscribe("/stop_obj_recog", 1, &restaurant_executor::stopObjCb, this);
  next_location_sub = nh_.subscribe("/locations", 1, &restaurant_executor::nextLocationCb, this);
  ordersCb = nh_.subscribe("/orders", 1, &restaurant_executor::orderCb, this);
  init_knowledge();
}

void restaurant_executor::init_knowledge()
{
  // definir un geometry_msgs::PoseStamped como punto init
  //MODIFICAR EL PUNTO INIT
  //addMapElement(0.0, 0.0, 0.0, -3.1416 / 2.0, "init");
  addMapElement(0.0, 0.0, 0.0, -3.1416 / 2.0, "entrance");
  addMapElement(1.8, -1.9, 0.0, 3.1416 / 2.0, "kitchen");
  addMapElement(1.39, 1.62, 0.0, 3.1416 / 2.0, "living room");
  addMapElement(4.23, 0.24, 0.0, -3.1416, "studio");
}

void restaurant_executor::Init_code_once()
{
  ROS_WARN("State Init");
  //Say that Restaurant starts
  std::string str = "Restaurant Starts";
  talk(str);
}

void restaurant_executor::navigate_to_init_code_iterative()
{
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
  it = locations_map.find("studio");
  //REVISAR QUE NO HAYA NULL POINTERS
  //ROS_INFO("%f %f", it->second.pose.position.x, it->second.pose.position.y);
  navigate_pub.publish(it->second);
}

void restaurant_executor::navigate_to_init_code_once()
{
  ROS_WARN("State Navigate_to_init");
}

void restaurant_executor::aproach_person_code_once()
{
  ROS_WARN("State Aproach person");
  //Anadir dependencia con person followed data
  //addDependency("Person_Followed_Publisher");
  addDependency("PD_Algorithm");
}
void restaurant_executor::searching_person_code_once()
{
  ROS_WARN("State Searching Person");
  loc_reached = false;
  std::string str = "I am looking for you";
  talk(str);
  addDependency("Person_Followed_Publisher");
}

void restaurant_executor::understand_object_code_once()
{
  ROS_WARN("State Understand Object");
  personSaw = false;
  removeDependency("PD_Algorithm");
  std::string str = "Do you want something?";
  talk(str);
  addDependency("order_DialogInterface");
}

void restaurant_executor::understand_location_code_once()
{
  ROS_WARN("The object is %s\n",
   object.c_str());
  ROS_WARN("understand location");
  removeDependency("order_DialogInterface");
  objUnderstood = false;
  std::string str = "So you want the " + object + ". Where I have to go?";
  talk(str);
  addDependency("location_DialogInterface");
}

void restaurant_executor::navigate_to_location_code_once()
{
  ROS_WARN("State navigate to location");
  removeDependency("location_DialogInterface");
  removeDependency("Person_Followed_Publisher");
  understanding_loc2go_loc = false;
  loc_reached = false;
  std::string str = "I am going to the " + nextLocation;
  talk(str);
}

void restaurant_executor::navigate_to_location_code_iterative()
{
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
  it = locations_map.find(nextLocation);
  if (it!=locations_map.end())
  {
    ROS_WARN("Going to %s\n", nextLocation.c_str());
    navigate_pub.publish(it->second);
  }
}

void restaurant_executor::navigate_to_end_code_once()
{
  ROS_WARN("State Navigate_to_end");
  removeDependency("Person_Followed_Publisher");
  removeDependency("PD_Algorithm");
  std::string str = "I founded the " + object;
  ROS_WARN("%s",str.c_str());
  talk(str);
}

void restaurant_executor::navigate_to_end_code_iterative()
{
  navigate_to_init_code_iterative();
}


void restaurant_executor::aproach_object_code_once()
{
  loc_reached = false;
  ROS_WARN("State Aproach object");
  std::string str = "I am going to approach";
  talk(str);
  addDependency("Person_Followed_Publisher");
  addDependency("PD_Algorithm");
}

void restaurant_executor::End_code_once()
{
  loc_reached = false;
  ROS_WARN("State End");
  std::string str = "Here you have the " + object;
  talk(str);
}

bool restaurant_executor::Init_2_navigate_to_init()
{
  return true;
}

bool restaurant_executor::searching_person_2_aproach_person()
{
  //return true;

  if (personSaw)
  {
    personSaw = false;
    return true;
  }
  return false;

}

bool restaurant_executor::navigate_to_init_2_searching_person()
{
  //return true;
  return loc_reached;
}

bool restaurant_executor::aproach_person_2_understand_object()
{
  //return true;
  return personSaw;
}

bool restaurant_executor::understand_object_2_understand_location()
{
  return objUnderstood;
}

bool restaurant_executor::understand_location_2_navigate_to_location()
{
  return understanding_loc2go_loc;
}

bool restaurant_executor::navigate_to_location_2_aproach_object()
{
  //return true;
  return loc_reached;
}

bool restaurant_executor::aproach_object_2_navigate_to_end()
{
  //return true;
  return personSaw;
}

bool restaurant_executor::navigate_to_end_2_End()
{
  return loc_reached;
}

void restaurant_executor::personDataCb(const robocuphomeeducation_msgs::PersonFollowedData::ConstPtr& msg)
{
  dist_to_person = msg->dist;
  //removeDependency("mover_publisher");
  personSaw = true;
}

void restaurant_executor::targetReachedCb(const std_msgs::Empty::ConstPtr& msg)
{
  ROS_WARN("targetReachedCb");
  loc_reached = true;
}

void restaurant_executor::stopObjCb(const std_msgs::Empty::ConstPtr& msg)
{
  objRecogFinished = true;
}

void restaurant_executor::orderCb(const std_msgs::String::ConstPtr& msg)
{
  object = msg->data;
  ROS_WARN("Me ha llegado el objeto");
  objUnderstood = true;
}

void restaurant_executor::nextLocationCb(const std_msgs::String::ConstPtr& msg)
{
  nextLocation = msg->data;
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
  it = locations_map.find(nextLocation);
  if(it != locations_map.end()){
    understanding_loc2go_loc = true;
    next_location_sub.shutdown();
  }
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

void restaurant_executor::talk(std::string str)
{
  std_msgs::String msg;
  msg.data = str;
  talk_pub.publish(msg);
}
