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
#ifndef OPEN_EXECUTOR_H_
#define OPEN_EXECUTOR_H_

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "open_HFSM.h"
#include <string>
#include <ctime>
#include "follow_person/PersonFollowedData.h"
#include "geometry_msgs/PoseStamped.h"
#include <tf2/LinearMath/Quaternion.h>

class Open_executor : public bica::open_HFSM
{
public:
  Open_executor();
  void init_knowledge();
  void navigate_to_loc_code_iterative();
  //void navigate_to_loc_code_once();
  void understand_excludes_nodes_code_iterative();
  void understand_excludes_nodes_code_once();
  //void rescue_teddy_bear_code_iterative();
  //void rescue_teddy_bear_code_once();
  //void get_dijkstra_code_iterative();
  void get_dijkstra_code_once();
  //void Init_code_iterative();
  void Init_code_once();
  //void understand_goal_code_iterative();
  void understand_goal_code_once();
  //void return_to_home_code_iterative();
  //void return_to_home_code_once();
  bool understand_excludes_nodes_2_get_dijkstra();
  bool get_dijkstra_2_navigate_to_loc();
  bool understand_goal_2_understand_excludes_nodes();
  //bool rescue_teddy_bear_2_return_to_home();
  bool Init_2_understand_goal();
  bool navigate_to_loc_2_rescue_teddy_bear();

  void addMapElement(float px, float py, float pz, float orientation, std::string key);
  void dijsktraOutCb(const std_msgs::String::ConstPtr& msg);
  void commandsCb(const std_msgs::String::ConstPtr& msg);
  void locsCb(const std_msgs::String::ConstPtr& msg);
  void targetReachedCb(const std_msgs::Empty::ConstPtr& msg);
  void talk(std::string str);

private:
  ros::NodeHandle nh_;
  ros::Subscriber pathSolver, locationsSub, commands_sub, loc_reached_sub;
  ros::Publisher locsPub, talk_pub, navigate_pub;
  int locsIterator;
  std::map<std::string, geometry_msgs::PoseStamped> locations_map;
  std::vector<geometry_msgs::PoseStamped> waypoints_to_visit;
  bool undersGoal2UndersExcludes, wps_ready, location_asked, dijsktra_rcv, goal_sended, loc_reached, nav_finished;
};

#endif
