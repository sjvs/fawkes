/***************************************************************************
 *  tabletop_switch_plugin.cpp - Plugin to switch on/off all tabletop plugins
 *
 *  Created: Fri May 23 11:56:42 2014
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

#include "tabletop_switch_thread.h"

using namespace fawkes;

/** Plugin to switch on/off all tabletop plugin
 * @author Till Hofmann
 */
class TabletopSwitchPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  TabletopSwitchPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new TabletopSwitchThread());
  }
};

PLUGIN_DESCRIPTION("Plugin to switch on/off all tabletop plugins")
EXPORT_PLUGIN(TabletopSwitchPlugin)
