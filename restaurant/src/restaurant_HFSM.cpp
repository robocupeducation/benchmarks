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

*   THIS SOFTWARE IS PROVrestaurant_HFSMED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCrestaurant_HFSMENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "restaurant_HFSM.h"

namespace bica
{
restaurant_HFSM::restaurant_HFSM() : state_(INIT), myBaseId_("restaurant_HFSM")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

restaurant_HFSM::~restaurant_HFSM()
{
}

void restaurant_HFSM::activateCode()
{
  	deactivateAllDeps();

	state_ = INIT;
	state_ts_ = ros::Time::now();

	Init_activateDeps();
	Init_code_once();

}

bool restaurant_HFSM::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case UNDERSTAND_OBJECT:

	understand_object_code_iterative();

	msg.data = "understand_object";
	if(understand_object_2_understand_location())
	{

	deactivateAllDeps();

	state_ = UNDERSTAND_LOCATION;
	state_ts_ = ros::Time::now();

	understand_location_activateDeps();
	understand_location_code_once();
	}
	state_pub_.publish(msg);
	break;

	case NAVIGATE_TO_END:

	navigate_to_end_code_iterative();

	msg.data = "navigate_to_end";
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

	case UNDERSTAND_LOCATION:

	understand_location_code_iterative();

	msg.data = "understand_location";
	if(understand_location_2_navigate_to_location())
	{

	deactivateAllDeps();

	state_ = NAVIGATE_TO_LOCATION;
	state_ts_ = ros::Time::now();

	navigate_to_location_activateDeps();
	navigate_to_location_code_once();
	}
	state_pub_.publish(msg);
	break;

	case NAVIGATE_TO_INIT:

	navigate_to_init_code_iterative();

	msg.data = "navigate_to_init";
	if(navigate_to_init_2_aproach_person())
	{

	deactivateAllDeps();

	state_ = APROACH_PERSON;
	state_ts_ = ros::Time::now();

	aproach_person_activateDeps();
	aproach_person_code_once();
	}
	state_pub_.publish(msg);
	break;

	case NAVIGATE_TO_LOCATION:

	navigate_to_location_code_iterative();

	msg.data = "navigate_to_location";
	if(navigate_to_location_2_aproach_object())
	{

	deactivateAllDeps();

	state_ = APROACH_OBJECT;
	state_ts_ = ros::Time::now();

	aproach_object_activateDeps();
	aproach_object_code_once();
	}
	state_pub_.publish(msg);
	break;

	case APROACH_OBJECT:

	aproach_object_code_iterative();

	msg.data = "aproach_object";
	if(aproach_object_2_navigate_to_end())
	{

	deactivateAllDeps();

	state_ = NAVIGATE_TO_END;
	state_ts_ = ros::Time::now();

	navigate_to_end_activateDeps();
	navigate_to_end_code_once();
	}
	state_pub_.publish(msg);
	break;

	case APROACH_PERSON:

	aproach_person_code_iterative();

	msg.data = "aproach_person";
	if(aproach_person_2_understand_object())
	{

	deactivateAllDeps();

	state_ = UNDERSTAND_OBJECT;
	state_ts_ = ros::Time::now();

	understand_object_activateDeps();
	understand_object_code_once();
	}
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
restaurant_HFSM::deactivateAllDeps()
{
};

void
restaurant_HFSM::understand_object_activateDeps()
{
}

void
restaurant_HFSM::navigate_to_end_activateDeps()
{
}

void
restaurant_HFSM::Init_activateDeps()
{
}

void
restaurant_HFSM::understand_location_activateDeps()
{
}

void
restaurant_HFSM::navigate_to_init_activateDeps()
{
}

void
restaurant_HFSM::navigate_to_location_activateDeps()
{
}

void
restaurant_HFSM::aproach_object_activateDeps()
{
}

void
restaurant_HFSM::aproach_person_activateDeps()
{
}



} /* namespace bica */
