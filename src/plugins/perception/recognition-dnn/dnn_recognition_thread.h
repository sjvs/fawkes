/***************************************************************************
 *  imgrec_thread.h - Thread to print recognized Dnn
 *
 *  Created: Su June 2 10:10:00 2018
 *  Copyright  2018 Daniel Habering
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


#ifndef __PLUGINS_IMAGE_RECOGNITION_THREAD_H_
#define __PLUGINS_IMAGE_RECOGNITION_THREAD_H_

#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/vision.h>
#include <aspect/tf.h>
#include <string>

// config handling
#include <config/change_handler.h>

#include <tiny_dnn/tiny_dnn.h>

namespace fawkes {
  class DnnRecognitionInterface;
}

class DnnRecognitionThread
: public fawkes::Thread,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::VisionAspect,
  public fawkes::ConfigurationChangeHandler,
  public fawkes::ClockAspect,
  public fawkes::TransformAspect
{
public:
  DnnRecognitionThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();
  
 private:
 void convert_image(const std::string &imagefilename, 
		 double minv,
		 double maxv,
		 int w,
		 int h,
		 tiny_dnn::vec_t &data);
 void recognize_image(const std::string &dictionary, const std::string &src_filename);
 fawkes::DnnRecognitionInterface *mps_rec_if_;

// config handling
  void config_value_erased(const char *path) {};
  void config_tag_changed(const char *new_tag) {};
  void config_comment_changed(const fawkes::Configuration::ValueIterator *v) {};
  void config_value_changed(const fawkes::Configuration::ValueIterator *v){}; 


};

#endif
