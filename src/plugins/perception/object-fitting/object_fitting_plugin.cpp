/***************************************************************************
 *  object_fitting_plugin.cpp - Object Fitting
 *
 *  Created: Mon Apr 28 15:30:42 2014
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

#include "object_fitting_thread.h"

using namespace fawkes;

/** Plugin to fit objects to known shapes (cylinders)
 * @author Till Hofmann
 */
class ObjectFittingPlugin : public fawkes::Plugin
{
 public:
  /** Constructor.
   * @param config Fawkes configuration
   */
  ObjectFittingPlugin(Configuration *config)
    : Plugin(config)
  {
    thread_list.push_back(new ObjectFittingThread());
  }
};

PLUGIN_DESCRIPTION("Fit objects to known shapes")
EXPORT_PLUGIN(ObjectFittingPlugin)
