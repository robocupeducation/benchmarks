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
#ifndef SPO_RECOGNITION_HFSM_H_
#define SPO_RECOGNITION_HFSM_H_

#include <bica/Component.h>
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <string>

namespace bica
{
class spo_recognition_HFSM : public bica::Component
{
public:
  spo_recognition_HFSM();
  virtual ~spo_recognition_HFSM();

  void activateCode();

  	virtual void aproach_person_code_iterative() {};
	virtual void aproach_person_code_once() {};
	virtual void object_recognition_code_iterative() {};
	virtual void object_recognition_code_once() {};
	virtual void Init_code_iterative() {};
	virtual void Init_code_once() {};
	virtual void answer_question_code_iterative() {};
	virtual void answer_question_code_once() {};
	virtual void turn_back_code_iterative() {};
	virtual void turn_back_code_once() {};
	virtual void End_code_iterative() {};
	virtual void End_code_once() {};

  	virtual bool aproach_person_2_answer_question() {return false;};
	virtual bool Init_2_turn_back() {return false;};
	virtual bool turn_back_2_aproach_person() {return false;};
	virtual bool object_recognition_2_End() {return false;};
	virtual bool answer_question_2_object_recognition() {return false;};


  bool ok();

protected:
  ros::Time state_ts_;

private:
  void step() {}

  	void deactivateAllDeps();
	void aproach_person_activateDeps();
	void object_recognition_activateDeps();
	void Init_activateDeps();
	void answer_question_activateDeps();
	void turn_back_activateDeps();
	void End_activateDeps();


  	static const int APROACH_PERSON = 0;
	static const int OBJECT_RECOGNITION = 1;
	static const int INIT = 2;
	static const int ANSWER_QUESTION = 3;
	static const int TURN_BACK = 4;
	static const int END = 5;


  int state_;

  std::string myBaseId_;
  ros::NodeHandle nh_;
  ros::Publisher state_pub_;
};

} /* namespace bica */

#endif /* SPO_RECOGNITION_HFSM_H_ */
