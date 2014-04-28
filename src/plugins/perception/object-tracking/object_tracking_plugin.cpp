/***************************************************************************
 *  object_tracking_plugin.cpp - Object Tracking Plugin
 *
 *  Created: Wed Apr 16 17:52:48 2014
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

#include "object_tracking_thread.h"

using namespace fawkes;

/** Plugin to track (tabletop) objects
 * @author Till Hofmann
 */
class ObjectTrackingPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  ObjectTrackingPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new ObjectTrackingThread());
  }
};

PLUGIN_DESCRIPTION("Track (tabletop) objects")
EXPORT_PLUGIN(ObjectTrackingPlugin)
