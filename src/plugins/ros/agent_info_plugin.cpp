
/***************************************************************************
 *  agent_info_plugin.cpp - Publish AgentInfo to ROS
 *
 *  Created: Thu Dec 11 15:51:42 2014
 *  Copyright  2014  Till Hofmann
 *
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

#include <core/plugin.h>

#include "agent_info_thread.h"

using namespace fawkes;

/** Plugin to publish AgentInfo to ROS
 * @author Till Hofmann
 */
class RosAgentInfoPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  RosAgentInfoPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new RosAgentInfoThread());
  }
};

PLUGIN_DESCRIPTION("ROS AgentInfo Plugin")
EXPORT_PLUGIN(RosAgentInfoPlugin)
