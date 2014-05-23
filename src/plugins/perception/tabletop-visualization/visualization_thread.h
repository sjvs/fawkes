
/***************************************************************************
 *  visualization_thread.h - Visualization via rviz
 *
 *  Created: Fri Nov 11 00:11:23 2011
 *  Copyright  2011  Tim Niemueller [www.niemueller.de]
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

#ifndef __PLUGINS_PERCEPTION_TABLETOP_VISUALIZATION_THREAD_H_
#define __PLUGINS_PERCEPTION_TABLETOP_VISUALIZATION_THREAD_H_

//#ifndef HAVE_VISUAL_DEBUGGING
//#  error TabletopVisualizationThread was disabled by build flags
//#endif

//#include "visualization_thread_base.h"

#include "../common/perception_common.h"
#include <core/threading/thread.h>
#include <core/threading/mutex.h>
#include <aspect/tf.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <aspect/syncpoint_manager.h>
#include <plugins/ros/aspect/ros.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <utils/time/time.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <list>
#include <string>

namespace fawkes {
  class SyncPoint;
  class SwitchInterface;
}

namespace ros {
  class Publisher;
}

class TabletopVisualizationThread
: public fawkes::Thread,
  public fawkes::BlockedTimingAspect,
  public fawkes::TransformAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::ROSAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::LoggingAspect,
  public fawkes::SyncPointManagerAspect
{
 public:

    /** Aligned vector of vectors/points. */
    typedef std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > V_Vector4f;
    /** aligned map of vectors. */
    typedef std::map<unsigned int, Eigen::Vector4f, std::less<unsigned int>,
        Eigen::aligned_allocator<std::pair<const unsigned int, Eigen::Vector4f>>>
        M_Vector4f;

    TabletopVisualizationThread();

    virtual void init();
    virtual void loop();
    virtual void finalize();

 private:
  fawkes::Mutex mutex_;
  std::list<fawkes::Position3DInterface *> obj_pos_ifs_;
  fawkes::Position3DInterface * table_pos_if_;
  ros::Publisher *vispub_;
#ifdef USE_POSEPUB
  ros::Publisher *posepub_;
#endif
  size_t last_id_num_;
  std::string frame_id_;

  fawkes::SwitchInterface *switch_if_;

  /* synchronization */
  fawkes::RefPtr<fawkes::SyncPoint> syncpoint_;

  unsigned int cfg_duration_;
  std::string cfg_object_name_pattern_;
  std::string cfg_syncpoint_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif
