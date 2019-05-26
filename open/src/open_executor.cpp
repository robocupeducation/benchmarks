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
  undersGoal2UndersExcludes = false;
  printf("%s\n", "Hola Mundo");
  talk_pub = nh_.advertise<std_msgs::String>("/talk", 1);
  pathSolver = nh_.subscribe("/solve", 1, &Open_executor::pathSolverCb, this);
  locationsSub = nh_.subscribe("/locations", 1, &Open_executor::locsCb, this);
  locsPub = nh_.advertise<std_msgs::String>("/dijsktra_inp", 1);
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
  addDependency("open_DialogInterface");
  ROS_WARN("[Understand goal] post dependency");
  std::string str = "Where I have to go?";
  talk(str);
  std_msgs::String s;
  s.data = "entrance";
  locsPub.publish(s);
}

void Open_executor::understand_excludes_nodes_code_once()
{
  ROS_WARN("State Understand Excludes Nodes");
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

void Open_executor::pathSolverCb(const std_msgs::String::ConstPtr& msg)
{
  std::string m = msg->data;
  ROS_WARN("%s", m.c_str());
}

void Open_executor::locsCb(const std_msgs::String::ConstPtr& msg)
{
  ROS_WARN("Callback executing");
  std::string str = msg->data;
  std::map<std::string, geometry_msgs::PoseStamped>::iterator it;
  it = locations_map.find(str);
  std::string answer;
  if(it != locations_map.end()){
    ROS_WARN("El destino existe");
    std_msgs::String s;
    s.data = str;
    locsPub.publish(s);
    undersGoal2UndersExcludes = true;
  }
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
