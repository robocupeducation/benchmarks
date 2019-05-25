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

*   THIS SOFTWARE IS PROVopen_HFSMED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCopen_HFSMENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "open_HFSM.h"

namespace bica
{
open_HFSM::open_HFSM() : state_(INIT), myBaseId_("open_HFSM")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

open_HFSM::~open_HFSM()
{
}

void open_HFSM::activateCode()
{
  	deactivateAllDeps();

	state_ = INIT;
	state_ts_ = ros::Time::now();

	Init_activateDeps();
	Init_code_once();

}

bool open_HFSM::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case NAVIGATE_TO_LOC:

	navigate_to_loc_code_iterative();

	msg.data = "navigate_to_loc";
	if(navigate_to_loc_2_rescue_teddy_bear())
	{

	deactivateAllDeps();

	state_ = RESCUE_TEDDY_BEAR;
	state_ts_ = ros::Time::now();

	rescue_teddy_bear_activateDeps();
	rescue_teddy_bear_code_once();
	}
	state_pub_.publish(msg);
	break;

	case UNDERSTAND_EXCLUDES_NODES:

	understand_excludes_nodes_code_iterative();

	msg.data = "understand_excludes_nodes";
	if(understand_excludes_nodes_2_get_dijkstra())
	{

	deactivateAllDeps();

	state_ = GET_DIJKSTRA;
	state_ts_ = ros::Time::now();

	get_dijkstra_activateDeps();
	get_dijkstra_code_once();
	}
	state_pub_.publish(msg);
	break;

	case RESCUE_TEDDY_BEAR:

	rescue_teddy_bear_code_iterative();

	msg.data = "rescue_teddy_bear";
	if(rescue_teddy_bear_2_return_to_home())
	{

	deactivateAllDeps();

	state_ = RETURN_TO_HOME;
	state_ts_ = ros::Time::now();

	return_to_home_activateDeps();
	return_to_home_code_once();
	}
	state_pub_.publish(msg);
	break;

	case GET_DIJKSTRA:

	get_dijkstra_code_iterative();

	msg.data = "get_dijkstra";
	if(get_dijkstra_2_navigate_to_loc())
	{

	deactivateAllDeps();

	state_ = NAVIGATE_TO_LOC;
	state_ts_ = ros::Time::now();

	navigate_to_loc_activateDeps();
	navigate_to_loc_code_once();
	}
	state_pub_.publish(msg);
	break;

	case INIT:

	Init_code_iterative();

	msg.data = "Init";
	if(Init_2_understand_goal())
	{

	deactivateAllDeps();

	state_ = UNDERSTAND_GOAL;
	state_ts_ = ros::Time::now();

	understand_goal_activateDeps();
	understand_goal_code_once();
	}
	state_pub_.publish(msg);
	break;

	case UNDERSTAND_GOAL:

	understand_goal_code_iterative();

	msg.data = "understand_goal";
	if(understand_goal_2_understand_excludes_nodes())
	{

	deactivateAllDeps();

	state_ = UNDERSTAND_EXCLUDES_NODES;
	state_ts_ = ros::Time::now();

	understand_excludes_nodes_activateDeps();
	understand_excludes_nodes_code_once();
	}
	state_pub_.publish(msg);
	break;

	case RETURN_TO_HOME:

	return_to_home_code_iterative();

	msg.data = "return_to_home";
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
open_HFSM::deactivateAllDeps()
{
};

void
open_HFSM::navigate_to_loc_activateDeps()
{
}

void
open_HFSM::understand_excludes_nodes_activateDeps()
{
}

void
open_HFSM::rescue_teddy_bear_activateDeps()
{
}

void
open_HFSM::get_dijkstra_activateDeps()
{
}

void
open_HFSM::Init_activateDeps()
{
}

void
open_HFSM::understand_goal_activateDeps()
{
}

void
open_HFSM::return_to_home_activateDeps()
{
}



} /* namespace bica */
