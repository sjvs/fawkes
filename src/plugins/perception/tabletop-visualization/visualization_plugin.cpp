/***************************************************************************
 *  visualization_plugin.cpp - Visualization via rviz
 *
 *  Created: Mon Apr 14 12:50:33 2014
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

#include "visualization_thread.h"

using namespace fawkes;

/** Plugin to visualize Tabletop Object Recognition
 * @author Till Hofmann
 */
class TabletopVisualizationPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  TabletopVisualizationPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new TabletopVisualizationThread());
  }
};

PLUGIN_DESCRIPTION("Visualize tabletop and object recognition")
EXPORT_PLUGIN(TabletopVisualizationPlugin)
