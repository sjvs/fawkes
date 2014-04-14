/***************************************************************************
 *  tabletop_recognition_plugin.cpp - Tabletop Recognition Plugin
 *
 *  Created: Thu Apr 10 14:31:56 2014
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

#include "tabletop_recognition_thread.h"
//#ifdef HAVE_VISUAL_DEBUGGING
//#  include "../tabletop-visualization/visualization_thread.h"
//#endif

using namespace fawkes;

/** Plugin to recognize a tabletop in a pointcloud input
 * This plugin is part of the tabletop object recognition plugins
 * @author Till Hofmann
 */
class TabletopRecognitionPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
    TabletopRecognitionPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new TabletopRecognitionThread());
//#ifdef HAVE_VISUAL_DEBUGGING
//    TabletopVisualizationThread *visthr = new TabletopVisualizationThread();
//    tabobjthr->set_visualization_thread(visthr);
//    thread_list.push_back(visthr);
//#endif
  }
};

PLUGIN_DESCRIPTION("Plugin to detect a tabletop in a pointcloud")
EXPORT_PLUGIN(TabletopRecognitionPlugin)
