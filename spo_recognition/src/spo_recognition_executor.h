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
#ifndef SPORECOGNITION_EXECUTOR_H_
#define SPORECOGNITION_EXECUTOR_H_

#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "spo_recognition_HFSM.h"
#include <string>
#include <ctime>
#include "robocuphomeeducation_msgs/PersonFollowedData.h"

class SPORecognition_executor : public bica::spo_recognition_HFSM
{
public:
  SPORecognition_executor();
  void init_knowledge();
  void Init_code_iterative();
  void Init_code_once();
  void object_recognition_code_iterative();
  void object_recognition_code_once();
  void aproach_person_code_iterative();
  void aproach_person_code_once();
  void turn_back_code_iterative();
  void turn_back_code_once();
  void answer_question_code_iterative();
  void answer_question_code_once();
  bool answer_question_2_object_recognition();
  bool aproach_person_2_answer_question();
  bool Init_2_turn_back();
  bool turn_back_2_aproach_person();

  //void End_code_iterative();
	void End_code_once();
  bool object_recognition_2_End();

  void talk(std::string str);
  void personDataCb(const robocuphomeeducation_msgs::PersonFollowedData::ConstPtr& msg);
  void objectCb(const std_msgs::String::ConstPtr& msg);
  void speakCb(const std_msgs::Empty::ConstPtr& msg);
  void stopObjCb(const std_msgs::Empty::ConstPtr& msg);

private:

  float minDist, maxDist, dist_to_person;

  ros::NodeHandle nh_;
  ros::Publisher talk_pub;
  ros::Subscriber personDataSub, objectSub, answerFinishedSub, objRecogFinishedSub;
  time_t beginTime, finishTime;
  std::string object;
  bool objArrived, answerFinished, objRecogFinished, personSaw;
};

#endif
