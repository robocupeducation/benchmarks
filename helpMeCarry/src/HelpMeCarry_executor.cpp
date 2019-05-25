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

HelpMeCarry_executor::HelpMeCarry_executor()
{
  i = 0;
  loc_reached = 0;
  understanding_loc2go_loc = 0;
  follow_person2understanding_location = 0;
  personSaw = 0;
  talk_pub = nh_.advertise<std_msgs::String>("/talk", 1);
  navigate_pub = nh_.advertise<geometry_msgs::PoseStamped>("/navigate_to", 1);
  loc_reached_sub = nh_.subscribe("/navigate_to/goal_reached", 1, &HelpMeCarry_executor::targetReachedCb, this);
  orders_sub = nh_.subscribe("/commands", 1, &HelpMeCarry_executor::orderCb, this);
  next_location_sub = nh_.subscribe("/locations", 1, &HelpMeCarry_executor::nextLocationCb, this);
  errorsSub = nh_.subscribe("/errors", 1, &HelpMeCarry_executor::errorCb, this);
  personDataSub = nh_.subscribe("/person_followed_data", 1, &HelpMeCarry_executor::personDataCb, this);
  init_knowledge();
}


void HelpMeCarry_executor::init_knowledge()
{
  // definir un geometry_msgs::PoseStamped como punto init
  addMapElement(0.0, 0.0, 0.0, -3.1416 / 2.0, "entrance");
  addMapElement(1.74, -1.84, 0.0, 3.1416, "kitchen");
  addMapElement(1.45, 1.8, 0.0, 0.0, "living room");
  addMapElement(4.23, 0.24, 0.0, -3.1416 / 2.0, "studio");

  // definir un map que asocie lugares a puntos de la casa geometry_msgs::PoseStamped y rellenar ese map

}

void HelpMeCarry_executor::navigate_to_loc_code_once()
{
  loc_reached = 0;
  ROS_WARN("State Navigate to Location");
  removeDependency("location_DialogInterface");
  understanding_loc2go_loc = 0;
  std::string str = "I am going to the " + nextLocation;
  talk(str);
}
void HelpMeCarry_executor::navigate_to_loc_code_iterative()
{
  ROS_WARN("Going to %s\n", nextLocation.c_str());
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
  it = locations_map.find(nextLocation);
  navigate_pub.publish(it->second);
}

void HelpMeCarry_executor::searching_person_code_iterative()
{

}

void HelpMeCarry_executor::searching_person_code_once()
{
  loc_reached = 0;
  ROS_WARN("State Searching_Person");
  std::string str = "I am looking for you";
  talk(str);
  ROS_WARN("State Searching_Person done");
  addDependency("mover_publisher");
  addDependency("Person_Followed_Publisher");
}

void HelpMeCarry_executor::follow_person_code_once()
{
  ROS_WARN("Follow Person");
  std::string str = "I am ready to follow you";
  talk(str);
  removeDependency("mover_publisher");
  addDependency("Person_Followed_Publisher");
  addDependency("PD_Algorithm");
  //addDependency("Person_Followed_Publisher");
  addDependency("commands_DialogInterface");
}

void HelpMeCarry_executor::Init_code_once()
{
  ROS_WARN("Init");
  //Say that Help me Carry starts
  std::string str = "Help me Carry Starts";
  talk(str);
  ROS_WARN("Init fiinished");
}

void HelpMeCarry_executor::navigate_to_init_code_once()
{

}

void HelpMeCarry_executor::navigate_to_init_code_iterative()
{
  ROS_WARN("navigate_to_init_code_iterative");
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
  it = locations_map.find("studio");
  if(it != locations_map.end()){
    ROS_INFO("%f %f", it->second.pose.position.x, it->second.pose.position.y);
    navigate_pub.publish(it->second);
  }
  else
  {
    ROS_ERROR("[navigate_to_init_code_iterative] Point doen's founded");
  }

}

void HelpMeCarry_executor::understanding_next_location_code_once()
{
  ROS_WARN("State Undestanding Next Location");
  removeDependency("PD_Algorithm");
  removeDependency("Person_Followed_Publisher");
  removeDependency("commands_DialogInterface");
  std::string str = "Where I have to go?";
  talk(str);
  addDependency("location_DialogInterface");
}

void HelpMeCarry_executor::understanding_next_location_code_iterative()
{

}

void HelpMeCarry_executor::End_code_once()
{
  std::string str = "Help me carry finished";
  talk(str);
}

bool HelpMeCarry_executor::Init_2_navigate_to_init()
{
  return true;
}


bool HelpMeCarry_executor::navigate_to_init_2_searching_person()
{
  return loc_reached;
  //return true;
}

bool HelpMeCarry_executor::searching_person_2_follow_person()
{
  return personSaw;
}

bool HelpMeCarry_executor::understanding_next_location_2_navigate_to_loc()
{
  return understanding_loc2go_loc;
}

bool HelpMeCarry_executor::follow_person_2_understanding_next_location()
{
  return follow_person2understanding_location;
}

bool HelpMeCarry_executor::navigate_to_loc_2_End()
{
  //return true;
  return loc_reached;
}

void HelpMeCarry_executor::targetReachedCb(const std_msgs::Empty::ConstPtr& msg)
{
  loc_reached = 1;
}

void HelpMeCarry_executor::nextLocationCb(const std_msgs::String::ConstPtr& msg)
{
  nextLocation = msg->data;
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
  it = locations_map.find(nextLocation);
  if(it != locations_map.end()){
    understanding_loc2go_loc = 1;
    next_location_sub.shutdown();
  }
}
void HelpMeCarry_executor::orderCb(const std_msgs::String::ConstPtr& msg)
{
  std::string str = msg->data;
  if(str == "Stop"){
    follow_person2understanding_location = true;
  }
}

void HelpMeCarry_executor::errorCb(const std_msgs::String::ConstPtr& msg)
{
  std::string str = msg->data;
  if(str == "unknown command"){
    str = "I dont unsdertand you, so I am going to the kitchen";
    talk(str);
    understanding_loc2go_loc = 1;
    nextLocation = "kitchen";
  }else{
    str = "There is no network, so i cannot understand you. I am going to the kitchen";
    talk(str);
    understanding_loc2go_loc = 1;
    nextLocation = "kitchen";
  }
}

void HelpMeCarry_executor::personDataCb(const follow_person::PersonFollowedData::ConstPtr& msg)
{
  dist_to_person = msg->dist;
  removeDependency("mover_publisher");
  personSaw = 1;
  //personDataSub.shutdown();
}

void HelpMeCarry_executor::addMapElement(float px, float py, float pz, float orientation, std::string key)
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

void HelpMeCarry_executor::talk(std::string str)
{
  std_msgs::String msg;
  msg.data = str;
  talk_pub.publish(msg);
}
