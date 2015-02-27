
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
#include <aspect/syncpoint.h>
#include <plugins/ros/aspect/ros.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>
#include <interfaces/TabletopHullInterface.h>
#include <interfaces/TabletopEdgesInterface.h>

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
  public fawkes::SyncPointAspect
{
 public:

    TabletopVisualizationThread();

    virtual void init();
    virtual void loop();
    virtual void finalize();

 private:
   visualization_msgs::Marker visualize_hull(fawkes::TabletopHullInterface* iface, uint &idnum);
   visualization_msgs::Marker visualize_cvxhull_vertices(fawkes::TabletopHullInterface* iface, uint &idnum);
   visualization_msgs::Marker visualize_plane(fawkes::TabletopHullInterface* iface, uint &idnum);
//   visualization_msgs::Marker visualize_frustrum(uint &idnum);
//   visualization_msgs::Marker visualize_frustrum_triangles(uint &idnum);

  void delete_all_markers();

 private:
  fawkes::Mutex mutex_;
  std::list<fawkes::Position3DInterface *> obj_pos_ifs_;
  fawkes::Position3DInterface * table_pos_if_;
  fawkes::TabletopHullInterface * hull_if_;
  fawkes::TabletopHullInterface *model_hull_if_;
  fawkes::TabletopEdgesInterface *good_hull_edges_if_;
  ros::Publisher *vispub_;
#ifdef USE_POSEPUB
  ros::Publisher *posepub_;
#endif
  size_t last_id_num_;
  std::string frame_id_;

  fawkes::SwitchInterface *switch_if_;

  unsigned int cfg_duration_;
  std::string cfg_object_name_pattern_;
  std::string cfg_syncpoint_;
  bool cfg_show_cvxhull_line_highlighting_;
  bool cfg_show_cvxhull_vertices_;
  bool cfg_show_cvxhull_vertex_ids_;
  bool cfg_show_frustrum_;
  float cfg_horizontal_va_;
  float cfg_vertical_va_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


#endif
