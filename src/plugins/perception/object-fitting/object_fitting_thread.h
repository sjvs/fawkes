/***************************************************************************
 *  object_fitting_thread.h - Object Fitting
 *
 *  Created: Mon Apr 28 15:32:17 2014
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

#ifndef __PLUGINS_PERCEPTION_OBJECT_FITTING_THREAD_H_
#define __PLUGINS_PERCEPTION_OBJECT_FITTING_THREAD_H_

// must be first for reliable ROS detection
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/logging.h>
#include <aspect/configurable.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/tf.h>
#include <aspect/pointcloud.h>
#include <aspect/syncpoint_manager.h>

#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>

#include <Eigen/StdVector>

#include <map>
#include <list>
#include <vector>

#include "../common/perception_common.h"


namespace fawkes {
  class Position3DInterface;
  class SyncPoint;
  class SwitchInterface;
}

class ObjectFittingThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::TransformAspect,
  public fawkes::PointCloudAspect,
  public fawkes::SyncPointManagerAspect
{
 public:
  ObjectFittingThread();
  virtual ~ObjectFittingThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  Eigen::Vector4f fit_cylinder(
    fawkes::perception::CloudConstPtr obj_in_base_frame,
    uint const &centroid_i);
  void set_position(fawkes::Position3DInterface *iface, bool is_visible,
    const Eigen::Vector4f &centroid);
  bool compute_bounding_box_scores(Eigen::Vector3f& cluster_dim,
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >& scores);
  double compute_similarity(double d1, double d2);

 private:
  std::map<uint, std::vector<double>> obj_likelihoods_;
  std::vector < Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > known_obj_dimensions_;
  std::map<unsigned int, double> obj_shape_confidence_;
  std::vector< Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f> > cylinder_params_;
  std::map<unsigned int, signed int> best_obj_guess_;

  std::vector<fawkes::Position3DInterface *> pos_ifs_;

  fawkes::SwitchInterface *switch_if_;

  /* synchronization */
  fawkes::RefPtr<fawkes::SyncPoint> syncpoint_in_;
  fawkes::RefPtr<fawkes::SyncPoint> syncpoint_out_;

  /* configuration */
  bool cfg_verbose_output_;
  std::string cfg_pointclouds_;
  std::string cfg_output_prefix_;
  bool cfg_use_colored_input_;
  std::string cfg_baselink_frame_id_;
  std::string cfg_syncpoint_in_;
  std::string cfg_syncpoint_out_;

  std::vector<fawkes::RefPtr<const pcl::PointCloud<fawkes::perception::PointType> > > finput_;
  std::vector<fawkes::perception::CloudConstPtr> input_;
  std::vector<fawkes::RefPtr<const pcl::PointCloud<fawkes::perception::ColorPointType> > > fcolored_input_;
  std::vector<fawkes::perception::ColorCloudConstPtr> colored_input_;

};


#endif
