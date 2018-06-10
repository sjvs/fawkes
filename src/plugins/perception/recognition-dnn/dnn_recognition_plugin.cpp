/***************************************************************************
 *  *  imgrec_plugin.cpp - Plugin for classifing images using a pretrained cnn
 *   *
 *    *  Created: Sun 2.June 10:0:00 2018
 *     *  Copyright  2018 Daniel Habering
 *      ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *   *  it under the terms of the GNU General Public License as published by
 *    *  the Free Software Foundation; either version 2 of the License, or
 *     *  (at your option) any later version.
 *      *
 *       *  This program is distributed in the hope that it will be useful,
 *        *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *         *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *          *  GNU Library General Public License for more details.
 *           *
 *            *  Read the full text in the LICENSE.GPL file in the doc directory.
 *             */

#include <core/plugin.h>

#include "dnn_recognition_thread.h"

using namespace fawkes;

class DnnRecognitionPlugin : public fawkes::Plugin
{
	 public:
	   /** Constructor.
	   *    * @param config Fawkes configuration
	   *       */
	    DnnRecognitionPlugin(Configuration *config)
	     : Plugin(config)
	    {
	       thread_list.push_back(new DnnRecognitionThread());
	    }
};
EXPORT_PLUGIN(DnnRecognitionPlugin)

PLUGIN_DESCRIPTION("Plugin for recognizing machines using the RealSense data")
	
