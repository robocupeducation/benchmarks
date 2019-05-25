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
#ifndef OPEN_HFSM_H_
#define OPEN_HFSM_H_

#include <bica/Component.h>
#include <ros/ros.h>

#include <std_msgs/String.h>

#include <string>

namespace bica
{
class open_HFSM : public bica::Component
{
public:
  open_HFSM();
  virtual ~open_HFSM();

  void activateCode();

  	virtual void navigate_to_loc_code_iterative() {};
	virtual void navigate_to_loc_code_once() {};
	virtual void understand_excludes_nodes_code_iterative() {};
	virtual void understand_excludes_nodes_code_once() {};
	virtual void rescue_teddy_bear_code_iterative() {};
	virtual void rescue_teddy_bear_code_once() {};
	virtual void get_dijkstra_code_iterative() {};
	virtual void get_dijkstra_code_once() {};
	virtual void Init_code_iterative() {};
	virtual void Init_code_once() {};
	virtual void understand_goal_code_iterative() {};
	virtual void understand_goal_code_once() {};
	virtual void return_to_home_code_iterative() {};
	virtual void return_to_home_code_once() {};

  	virtual bool understand_excludes_nodes_2_get_dijkstra() {return false;};
	virtual bool get_dijkstra_2_navigate_to_loc() {return false;};
	virtual bool understand_goal_2_understand_excludes_nodes() {return false;};
	virtual bool rescue_teddy_bear_2_return_to_home() {return false;};
	virtual bool Init_2_understand_goal() {return false;};
	virtual bool navigate_to_loc_2_rescue_teddy_bear() {return false;};


  bool ok();

protected:
  ros::Time state_ts_;

private:
  void step() {}

  	void deactivateAllDeps();
	void navigate_to_loc_activateDeps();
	void understand_excludes_nodes_activateDeps();
	void rescue_teddy_bear_activateDeps();
	void get_dijkstra_activateDeps();
	void Init_activateDeps();
	void understand_goal_activateDeps();
	void return_to_home_activateDeps();


  	static const int NAVIGATE_TO_LOC = 0;
	static const int UNDERSTAND_EXCLUDES_NODES = 1;
	static const int RESCUE_TEDDY_BEAR = 2;
	static const int GET_DIJKSTRA = 3;
	static const int INIT = 4;
	static const int UNDERSTAND_GOAL = 5;
	static const int RETURN_TO_HOME = 6;


  int state_;

  std::string myBaseId_;
  ros::NodeHandle nh_;
  ros::Publisher state_pub_;
};

} /* namespace bica */

#endif /* OPEN_HFSM_H_ */
