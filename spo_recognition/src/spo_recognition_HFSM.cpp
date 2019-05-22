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

*   THIS SOFTWARE IS PROVspo_recognition_HFSMED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*   FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*   COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*   INCspo_recognition_HFSMENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*   BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*   CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*   POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Francisco Martín fmrico@gmail.com */

/* Mantainer: Francisco Martín fmrico@gmail.com */

#include "spo_recognition_HFSM.h"

namespace bica
{
spo_recognition_HFSM::spo_recognition_HFSM() : state_(INIT), myBaseId_("spo_recognition_HFSM")
{
  state_ts_ = ros::Time::now();
  state_pub_ = nh_.advertise<std_msgs::String>("/" + myBaseId_ + "/state", 1, false);
}

spo_recognition_HFSM::~spo_recognition_HFSM()
{
}

void spo_recognition_HFSM::activateCode()
{
  	deactivateAllDeps();

	state_ = INIT;
	state_ts_ = ros::Time::now();

	Init_activateDeps();
	Init_code_once();

}

bool spo_recognition_HFSM::ok()
{
  if (active_)
  {
    std_msgs::String msg;

    switch (state_)
    {
      	case APROACH_PERSON:

	aproach_person_code_iterative();

	msg.data = "aproach_person";
	if(aproach_person_2_answer_question())
	{

	deactivateAllDeps();

	state_ = ANSWER_QUESTION;
	state_ts_ = ros::Time::now();

	answer_question_activateDeps();
	answer_question_code_once();
	}
	state_pub_.publish(msg);
	break;

	case OBJECT_RECOGNITION:

	object_recognition_code_iterative();

	msg.data = "object_recognition";
	if(object_recognition_2_End())
	{

	deactivateAllDeps();

	state_ = END;
	state_ts_ = ros::Time::now();

	End_activateDeps();
	End_code_once();
	}
	state_pub_.publish(msg);
	break;

	case INIT:

	Init_code_iterative();

	msg.data = "Init";
	if(Init_2_turn_back())
	{

	deactivateAllDeps();

	state_ = TURN_BACK;
	state_ts_ = ros::Time::now();

	turn_back_activateDeps();
	turn_back_code_once();
	}
	state_pub_.publish(msg);
	break;

	case ANSWER_QUESTION:

	answer_question_code_iterative();

	msg.data = "answer_question";
	if(answer_question_2_object_recognition())
	{

	deactivateAllDeps();

	state_ = OBJECT_RECOGNITION;
	state_ts_ = ros::Time::now();

	object_recognition_activateDeps();
	object_recognition_code_once();
	}
	state_pub_.publish(msg);
	break;

	case TURN_BACK:

	turn_back_code_iterative();

	msg.data = "turn_back";
	if(turn_back_2_aproach_person())
	{

	deactivateAllDeps();

	state_ = APROACH_PERSON;
	state_ts_ = ros::Time::now();

	aproach_person_activateDeps();
	aproach_person_code_once();
	}
	state_pub_.publish(msg);
	break;

	case END:

	End_code_iterative();

	msg.data = "End";
	state_pub_.publish(msg);
	break;


    }
  }

  return Component::ok();
}

void
spo_recognition_HFSM::deactivateAllDeps()
{
};

void
spo_recognition_HFSM::aproach_person_activateDeps()
{
}

void
spo_recognition_HFSM::object_recognition_activateDeps()
{
}

void
spo_recognition_HFSM::Init_activateDeps()
{
}

void
spo_recognition_HFSM::answer_question_activateDeps()
{
}

void
spo_recognition_HFSM::turn_back_activateDeps()
{
}

void
spo_recognition_HFSM::End_activateDeps()
{
}



} /* namespace bica */
