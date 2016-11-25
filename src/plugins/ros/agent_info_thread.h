
/***************************************************************************
 *  agent_info_thread.h - Publish AgentInfo to ROS
 *
 *  Created: Wed Sep 25 18:27:26 2013
 *  Copyright  2013  Till Hofmann
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __PLUGINS_ROS_JOINT_THREAD_H_
#define __PLUGINS_ROS_JOINT_THREAD_H_

// TODO check includes
#include <core/threading/thread.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <plugins/ros/aspect/ros.h>
#include <blackboard/interface_listener.h>
#include <core/threading/mutex.h>
#include <interfaces/AgentInterface.h>

#include <list>

// from ROS
#include <ros/node_handle.h>

#include <continual_planning_msgs/ContinualPlanningStatus.h>

class RosAgentInfoThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::ROSAspect,
  public fawkes::BlackBoardInterfaceListener
{
 public:
  RosAgentInfoThread();
  virtual ~RosAgentInfoThread();

  virtual void init();
  virtual void finalize();

  virtual void bb_interface_data_changed(fawkes::Interface *interface) throw();

 private:
  void publish_plan(std::string plan, continual_planning_msgs::ContinualPlanningStatus::_component_type);
 private:
  ros::Publisher ros_pub_planner_;
  ros::Publisher ros_pub_agent_message_;
  fawkes::AgentInterface * agent_if_;
  // remember the last message to check whether the message was actually updated
  std::string last_agent_message_;

};

#endif
