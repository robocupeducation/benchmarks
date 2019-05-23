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

*   THIS SOFTWARE IS PROVHelpMeCarry_HFSMED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCHelpMeCarry_HFSMENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "HelpMeCarry_HFSM.h"

namespace bica
{
HelpMeCarry_HFSM::HelpMeCarry_HFSM() : state_(INIT), myBaseId_("HelpMeCarry_HFSM")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

HelpMeCarry_HFSM::~HelpMeCarry_HFSM()
{
}

void HelpMeCarry_HFSM::activateCode()
{
  	deactivateAllDeps();

	state_ = INIT;
	state_ts_ = ros::Time::now();

	Init_activateDeps();
	Init_code_once();

}

bool HelpMeCarry_HFSM::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case UNDERSTANDING_NEXT_LOCATION:

	understanding_next_location_code_iterative();

	msg.data = "understanding_next_location";
	if(understanding_next_location_2_navigate_to_loc())
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
	if(Init_2_navigate_to_init())
	{

	deactivateAllDeps();

	state_ = NAVIGATE_TO_INIT;
	state_ts_ = ros::Time::now();

	navigate_to_init_activateDeps();
	navigate_to_init_code_once();
	}
	state_pub_.publish(msg);
	break;

	case END:

	End_code_iterative();

	msg.data = "End";
	state_pub_.publish(msg);
	break;

	case NAVIGATE_TO_LOC:

	navigate_to_loc_code_iterative();

	msg.data = "navigate_to_loc";
	if(navigate_to_loc_2_End())
	{

	deactivateAllDeps();

	state_ = END;
	state_ts_ = ros::Time::now();

	End_activateDeps();
	End_code_once();
	}
	state_pub_.publish(msg);
	break;

	case NAVIGATE_TO_INIT:

	navigate_to_init_code_iterative();

	msg.data = "navigate_to_init";
	if(navigate_to_init_2_follow_person())
	{

	deactivateAllDeps();

	state_ = FOLLOW_PERSON;
	state_ts_ = ros::Time::now();

	follow_person_activateDeps();
	follow_person_code_once();
	}
	state_pub_.publish(msg);
	break;

	case FOLLOW_PERSON:

	follow_person_code_iterative();

	msg.data = "follow_person";
	if(follow_person_2_understanding_next_location())
	{

	deactivateAllDeps();

	state_ = UNDERSTANDING_NEXT_LOCATION;
	state_ts_ = ros::Time::now();

	understanding_next_location_activateDeps();
	understanding_next_location_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
HelpMeCarry_HFSM::deactivateAllDeps()
{
};

void
HelpMeCarry_HFSM::understanding_next_location_activateDeps()
{
}

void
HelpMeCarry_HFSM::Init_activateDeps()
{
}

void
HelpMeCarry_HFSM::End_activateDeps()
{
}

void
HelpMeCarry_HFSM::navigate_to_loc_activateDeps()
{
}

void
HelpMeCarry_HFSM::navigate_to_init_activateDeps()
{
}

void
HelpMeCarry_HFSM::follow_person_activateDeps()
{
}



} /* namespace bica */
