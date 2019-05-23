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
#include "spo_recognition_executor.h"

SPORecognition_executor::SPORecognition_executor()
{
  talk_pub = nh_.advertise<std_msgs::String>("/talk", 1);
  personDataSub = nh_.subscribe("/person_followed_data", 1, &SPORecognition_executor::personDataCb, this);
  objectSub = nh_.subscribe("/object_detected", 1, &SPORecognition_executor::objectCb, this);
  answerFinishedSub = nh_.subscribe("/finish_speak", 1, &SPORecognition_executor::speakCb, this);
  objRecogFinishedSub = nh_.subscribe("/stop_obj_recog", 1, &SPORecognition_executor::stopObjCb, this);
  init_knowledge();
}


void SPORecognition_executor::init_knowledge()
{
  minDist = 900.0;
  maxDist = 1200.0;
  objArrived = false;
  answerFinished = false;
  objRecogFinished = false;
}

void SPORecognition_executor::Init_code_once()
{
  ROS_WARN("Init");
  //Say that Help me Carry starts
  std::string str = "Speech Object and Person Recognition starts";
  talk(str);
}

void SPORecognition_executor::Init_code_iterative()
{

}

void SPORecognition_executor::turn_back_code_once()
{
  // tomamos la marca de tiempo
  beginTime = time(NULL);
  addDependency("mover_publisher");
}

void SPORecognition_executor::turn_back_code_iterative()
{
  //publicar para que el robot gire
  ROS_WARN("State turn_back_code_iterative");
  finishTime = time(NULL);
}

void SPORecognition_executor::aproach_person_code_iterative()
{

}
void SPORecognition_executor::aproach_person_code_once()
{
  ROS_WARN("State Aproach_Person");
  removeDependency("mover_publisher");
  addDependency("Person_Followed_Publisher");
  addDependency("PD_Algorithm");
}

void SPORecognition_executor::object_recognition_code_iterative()
{
    ROS_WARN("State Object_Recognition iterative");
  if(objArrived){
      ROS_WARN("State Object_Recognition objArrived detected %s", object.c_str());
    talk(object);
  }
  objArrived = false;
}
void SPORecognition_executor::object_recognition_code_once()
{
  ROS_WARN("State Object_Recognition");
  removeDependency("main_DialogInterface");
  answerFinished = false;
  addDependency("Objects_Detector");
}

void SPORecognition_executor::answer_question_code_iterative()
{

}
void SPORecognition_executor::answer_question_code_once()
{
  //No se lo que tengo que hacer
  ROS_WARN("State Answer_Questions");
  removeDependency("Person_Followed_Publisher");
  removeDependency("PD_Algorithm");
  addDependency("main_DialogInterface");
}

void SPORecognition_executor::End_code_once()
{
  std::string str = "Speech, object and person recognition finished";
  talk(str);
}

bool SPORecognition_executor::Init_2_turn_back()
{
  return true;
}

bool SPORecognition_executor::turn_back_2_aproach_person()
{
  //checkear cuanto tiempo ha pasado desde que le mandamos que girara la primera vez
  return finishTime - beginTime >= 4.5;
}

bool SPORecognition_executor::aproach_person_2_answer_question()
{
  return dist_to_person >= minDist && dist_to_person <= maxDist;
}

bool SPORecognition_executor::answer_question_2_object_recognition()
{
  return answerFinished;
}

bool SPORecognition_executor::object_recognition_2_End()
{
  return objRecogFinished;
}

void SPORecognition_executor::stopObjCb(const std_msgs::Empty::ConstPtr& msg)
{
  objRecogFinished = true;
}

void SPORecognition_executor::personDataCb(const follow_person::PersonFollowedData::ConstPtr& msg)
{
  dist_to_person = msg->dist;
  personDataSub.shutdown();
}

void SPORecognition_executor::objectCb(const std_msgs::String::ConstPtr& msg)
{
  object = msg->data;
  objArrived = true;
}

void SPORecognition_executor::speakCb(const std_msgs::Empty::ConstPtr& msg)
{
  answerFinished = true;
}

void SPORecognition_executor::talk(std::string str)
{
  std_msgs::String msg;
  msg.data = str;
  talk_pub.publish(msg);
}
