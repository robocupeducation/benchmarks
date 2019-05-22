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
#ifndef HELPMECARRY_HFSM_H_
#define HELPMECARRY_HFSM_H_

#include <bica/Component.h>
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <string>

namespace bica
{
class HelpMeCarry_HFSM : public bica::Component
{
public:
  HelpMeCarry_HFSM();
  virtual ~HelpMeCarry_HFSM();

  void activateCode();

  	virtual void understanding_next_location_code_iterative() {};
	virtual void understanding_next_location_code_once() {};
	virtual void Init_code_iterative() {};
	virtual void Init_code_once() {};
	virtual void End_code_iterative() {};
	virtual void End_code_once() {};
	virtual void navigate_to_loc_code_iterative() {};
	virtual void navigate_to_loc_code_once() {};
	virtual void navigate_to_init_code_iterative() {};
	virtual void navigate_to_init_code_once() {};
	virtual void follow_person_code_iterative() {};
	virtual void follow_person_code_once() {};

  	virtual bool follow_person_2_understanding_next_location() {return false;};
	virtual bool Init_2_navigate_to_init() {return false;};
	virtual bool navigate_to_init_2_follow_person() {return false;};
	virtual bool navigate_to_loc_2_End() {return false;};
	virtual bool understanding_next_location_2_navigate_to_loc() {return false;};


  bool ok();

protected:
  ros::Time state_ts_;

private:
  void step() {}

  	void deactivateAllDeps();
	void understanding_next_location_activateDeps();
	void Init_activateDeps();
	void End_activateDeps();
	void navigate_to_loc_activateDeps();
	void navigate_to_init_activateDeps();
	void follow_person_activateDeps();


  	static const int UNDERSTANDING_NEXT_LOCATION = 0;
	static const int INIT = 1;
	static const int END = 2;
	static const int NAVIGATE_TO_LOC = 3;
	static const int NAVIGATE_TO_INIT = 4;
	static const int FOLLOW_PERSON = 5;


  int state_;

  std::string myBaseId_;
  ros::NodeHandle nh_;
  ros::Publisher state_pub_;
};

} /* namespace bica */

#endif /* HELPMECARRY_HFSM_H_ */
