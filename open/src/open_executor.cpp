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
#include "open_executor.h"

Open_executor::Open_executor()
{
  locsIterator = 0;
  undersGoal2UndersExcludes = false, wps_ready = false, dijsktra_rcv = false, loc_reached = false, nav_finished = false;;
  talk_pub = nh_.advertise<std_msgs::String>("/talk", 1);
  pathSolver = nh_.subscribe("/dijsktra_out", 1, &Open_executor::dijsktraOutCb, this);
  locationsSub = nh_.subscribe("/locations", 1, &Open_executor::locsCb, this);
  commands_sub = nh_.subscribe("/commands", 1, &Open_executor::commandsCb, this);
  locsPub = nh_.advertise<std_msgs::String>("/dijsktra_inp", 1);
  navigate_pub = nh_.advertise<geometry_msgs::PoseStamped>("/navigate_to", 1);
  loc_reached_sub = nh_.subscribe("/navigate_to/goal_reached", 1, &Open_executor::targetReachedCb, this);
  init_knowledge();
}


void Open_executor::init_knowledge()
{
  //cargar mapa
  addMapElement(0.0, 0.0, 0.0, 0.0, "entrance");
  addMapElement(2.32, -0.67, 0.0, 3.1416 / 2.0, "kitchen");
  addMapElement(2.98, 1.62, 0.0, 3.1416 / 2.0, "living room");
  addMapElement(1.62, 1.61, 0.0, -3.1416, "studio");
  addMapElement(3.2, -0.05, 0.0, -3.1416, "bath room");

}

void Open_executor::Init_code_once()
{
  ROS_WARN("State Init");
  std::string str = "Init state started";
  talk(str);
}

void Open_executor::understand_goal_code_once()
{
  ROS_WARN("State Understand Goal");
  addDependency("location_DialogInterface");
  std::string str = "Where I have to go?";
  talk(str);
  std_msgs::String s;
  s.data = "entrance";
  locsPub.publish(s);
}

void Open_executor::understand_excludes_nodes_code_once()
{
  ROS_WARN("State Understand Excludes Nodes");
  addDependency("commands_DialogInterface");
  location_asked = false;
}

void Open_executor::understand_excludes_nodes_code_iterative()
{
  ROS_WARN("understand_excludes_nodes");
  if (!location_asked)
  {
    std::string str = "Tell me the forbidden rooms";
    talk(str);
    location_asked = true;
  }
}

void Open_executor::get_dijkstra_code_once()
{
  ROS_WARN("State get_dijkstra_code");
  removeDependency("location_DialogInterface");
  removeDependency("commands_DialogInterface");
}


void Open_executor::navigate_to_loc_code_iterative()
{
  ROS_WARN("State navigate_to_loc_code_iterative");
  if (locsIterator == waypoints_to_visit.size())
  {
    nav_finished = true;
    return;
  }
  if (!goal_sended)
  {
    navigate_pub.publish(waypoints_to_visit[locsIterator]);
    locsIterator++;
    goal_sended = true;
  }
}


bool Open_executor::Init_2_understand_goal()
{
  return true;
}

bool Open_executor::understand_goal_2_understand_excludes_nodes()
{
  if(undersGoal2UndersExcludes){
    undersGoal2UndersExcludes = false;
    return true;
  }else{
    return false;
  }
}

bool Open_executor::understand_excludes_nodes_2_get_dijkstra()
{
  return wps_ready;
}

bool Open_executor::get_dijkstra_2_navigate_to_loc()
{
  return dijsktra_rcv;
}

bool Open_executor::navigate_to_loc_2_rescue_teddy_bear()
{
  return nav_finished;
}

void Open_executor::dijsktraOutCb(const std_msgs::String::ConstPtr& msg)
{
  ROS_WARN("dijsktraOutCb %s", msg->data.c_str());
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
  it = locations_map.find(msg->data);
  if(it != locations_map.end()){
    waypoints_to_visit.push_back(it->second);
  }
  else if(msg->data == "end")
  {
    dijsktra_rcv = true;
  }

}

void Open_executor::locsCb(const std_msgs::String::ConstPtr& msg)
{
  ROS_WARN("Callback executing");
  std::string str = msg->data;
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
  it = locations_map.find(str);
  std::string answer;
  if(it != locations_map.end()){
    ROS_WARN("[locsCb] location founded");
    std_msgs::String s;
    s.data = str;
    locsPub.publish(s);
    location_asked = false;
    undersGoal2UndersExcludes = true;
  }
}

void Open_executor::commandsCb(const std_msgs::String::ConstPtr& msg)
{
  std::string str = msg->data;
  if(str == "Stop"){
    str = "Ok, waypoints saved";
    talk(str);
    wps_ready = true;
  }
}

void Open_executor::targetReachedCb(const std_msgs::Empty::ConstPtr& msg)
{
  loc_reached = true;
  goal_sended = false;
}

void Open_executor::addMapElement(float px, float py, float pz, float orientation, std::string key)
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

void Open_executor::talk(std::string str)
{
  std_msgs::String msg;
  msg.data = str;
  talk_pub.publish(msg);
}
