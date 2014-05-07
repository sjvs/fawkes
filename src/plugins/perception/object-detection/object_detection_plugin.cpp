/***************************************************************************
 *  object_detection_plugin.cpp - Object Detection Plugin
 *
 *  Created: Tue Apr 15 16:59:22 2014
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

#include "object_detection_thread.h"

using namespace fawkes;

/** Plugin to detect objects in a pointcloud
 * This plugin is part of the tabletop object detection plugins
 * @author Till Hofmann
 */
class ObjectDetectionPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  ObjectDetectionPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new ObjectDetectionThread());
  }
};

PLUGIN_DESCRIPTION("Plugin to detect objects in a pointcloud")
EXPORT_PLUGIN(ObjectDetectionPlugin)
