
/***************************************************************************
 *  agent_info_thread.cpp - Publish AgentInfo to ROS
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

#include "agent_info_thread.h"

#include <ros/this_node.h>
#include <continual_planning_msgs/ContinualPlanningStatus.h>
#include <std_msgs/String.h>
#include <algorithm>

using namespace fawkes;
using namespace std;

/** @class RosAgentInfoThread "agent_info_thread.h"
 * Thread to publish Agent information to ROS.
 * This thread reads agent info from the blackboard and publishes it to ROS.
 * @author Till Hofmann
 */

/** Constructor. */
RosAgentInfoThread::RosAgentInfoThread()
  : Thread("RosAgentInfoThread", Thread::OPMODE_WAITFORWAKEUP),
    BlackBoardInterfaceListener("RosAgentInfoThread")
{
}

/** Destructor. */
RosAgentInfoThread::~RosAgentInfoThread()
{
}

void
RosAgentInfoThread::init()
{
  last_agent_message_ = "";
  ros_pub_planner_ = rosnode->advertise<continual_planning_msgs::ContinualPlanningStatus>("continual_planning_status", 10);
  ros_pub_agent_message_ = rosnode->advertise<std_msgs::String>("agent_info", 10);
  // open interface and listen for changed data
  agent_if_ = blackboard->open_for_reading<AgentInterface>("Agent");
  bbil_add_data_interface(agent_if_);
  
  // register to blackboard
  blackboard->register_listener(this);
}

void
RosAgentInfoThread::finalize()
{
  blackboard->unregister_listener(this);
  blackboard->close(agent_if_);
  ros_pub_planner_.shutdown();
  ros_pub_agent_message_.shutdown();
}

void
RosAgentInfoThread::bb_interface_data_changed(Interface *interface) throw()
{
  AgentInterface *iface = dynamic_cast<AgentInterface *>(interface);
  if (!iface) return;
  logger->log_debug(name(), "AgentInterface was updated");
  iface->read();

  // append history and plan to create the whole plan which is expected by
  // hybris_monitor
  string history = iface->history();
  // make sure last character is a separator
  if (history.back() != ';') {
    history.push_back(';');
  }
  publish_plan(history + iface->plan(), continual_planning_msgs::ContinualPlanningStatus::PLANNING);

  publish_plan(iface->plan(), continual_planning_msgs::ContinualPlanningStatus::CURRENT_PLAN);

  if (iface->message() != last_agent_message_) {
    std_msgs::String message;
    message.data = iface->message();
    ros_pub_agent_message_.publish(message);
    last_agent_message_ = iface->message();
  }
}

void
RosAgentInfoThread::publish_plan(string plan, continual_planning_msgs::ContinualPlanningStatus::_component_type component) {
  continual_planning_msgs::ContinualPlanningStatus status;
  status.component = component;
  replace(plan.begin(), plan.end(), ';', '\n');
  status.description = plan;
  ros_pub_planner_.publish(status);
}

