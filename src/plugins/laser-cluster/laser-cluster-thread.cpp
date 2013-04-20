
/***************************************************************************
 *  laser-cluster-thread.cpp - Thread to detect a cluster in 2D laser data
 *
 *  Created: Sun Apr 21 01:27:10 2013
 *  Copyright  2011-2013  Tim Niemueller [www.niemueller.de]
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

#include "laser-cluster-thread.h"
#include "cluster_colors.h"

#include <pcl_utils/utils.h>
#include <pcl_utils/comparisons.h>
#include <utils/time/wait.h>
#include <utils/math/angle.h>
#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>

#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>

#include <iostream>
using namespace std;

#define CFG_PREFIX "/laser-cluster/"

/** @class LaserClusterThread "laser-cluster-thread.h"
 * Main thread of laser-cluster plugin.
 * @author Tim Niemueller
 */

using namespace fawkes;


#ifdef USE_TIMETRACKER
#define TIMETRACK_START(c)                      \
  tt_->ping_start(c);				\

#define TIMETRACK_INTER(c1, c2)			\
 tt_->ping_end(c1);				\
 tt_->ping_start(c2);

#define TIMETRACK_END(c)			\
  tt_->ping_end(c);

#define TIMETRACK_ABORT(c)                      \
  tt_->ping_abort(c);

#else

#define TIMETRACK_START(c)
#define TIMETRACK_INTER(c1, c2)
#define TIMETRACK_END(c)
#define TIMETRACK_ABORT(c)
#endif

/** Constructor. */
LaserClusterThread::LaserClusterThread()
  : Thread("LaserClusterThread", Thread::OPMODE_CONTINUOUS),
    TransformAspect(TransformAspect::ONLY_LISTENER)
{
}


/** Destructor. */
LaserClusterThread::~LaserClusterThread()
{
}


void
LaserClusterThread::init()
{
  cfg_depth_filter_min_x_ = config->get_float(CFG_PREFIX"depth_filter_min_x");
  cfg_depth_filter_max_x_ = config->get_float(CFG_PREFIX"depth_filter_max_x");
  cfg_segm_max_iterations_ =
    config->get_uint(CFG_PREFIX"line_segmentation_max_iterations");
  cfg_segm_distance_threshold_ =
    config->get_float(CFG_PREFIX"line_segmentation_distance_threshold");
  cfg_segm_inlier_quota_ =
    config->get_float(CFG_PREFIX"line_segmentation_inlier_quota");
  cfg_cluster_tolerance_     = config->get_float(CFG_PREFIX"cluster_tolerance");
  cfg_cluster_min_size_      = config->get_uint(CFG_PREFIX"cluster_min_size");
  cfg_cluster_max_size_      = config->get_uint(CFG_PREFIX"cluster_max_size");
  cfg_input_pcl_             = config->get_string(CFG_PREFIX"input_cloud");
  cfg_result_frame_          = config->get_string(CFG_PREFIX"result_frame");

  finput_ = pcl_manager->get_pointcloud<PointType>(cfg_input_pcl_.c_str());
  input_ = pcl_utils::cloudptr_from_refptr(finput_);

  try {
    double rotation[4] = {0., 0., 0., 1.};
    cluster_pos_if_ = NULL;
    cluster_pos_if_ = blackboard->open_for_writing<Position3DInterface>("Laser Cluster");
    cluster_pos_if_->set_rotation(rotation);
    cluster_pos_if_->write();

    switch_if_ = NULL;
    switch_if_ = blackboard->open_for_writing<SwitchInterface>("laser-cluster");
    switch_if_->set_enabled(true);
    switch_if_->write();
  } catch (Exception &e) {
    blackboard->close(cluster_pos_if_);
    blackboard->close(switch_if_);
    throw;
  }

  fclusters_ = new pcl::PointCloud<ColorPointType>();
  fclusters_->header.frame_id = finput_->header.frame_id;
  fclusters_->is_dense = false;
  pcl_manager->add_pointcloud<ColorPointType>("laser-cluster", fclusters_);
  clusters_ = pcl_utils::cloudptr_from_refptr(fclusters_);

  seg_.setOptimizeCoefficients(true);
  seg_.setModelType(pcl::SACMODEL_LINE);
  seg_.setMethodType(pcl::SAC_RANSAC);
  seg_.setMaxIterations(cfg_segm_max_iterations_);
  seg_.setDistanceThreshold(cfg_segm_distance_threshold_);

  loop_count_ = 0;

#ifdef USE_TIMETRACKER
  tt_ = new TimeTracker();
  tt_loopcount_ = 0;
  ttc_full_loop_          = tt_->add_class("Full Loop");
  ttc_msgproc_            = tt_->add_class("Message Processing");
  ttc_extract_lines_      = tt_->add_class("Line Extraction");
  ttc_clustering_         = tt_->add_class("Clustering");
#endif
}


void
LaserClusterThread::finalize()
{
  input_.reset();
  clusters_.reset();

  pcl_manager->remove_pointcloud("laser-cluster");
  
  blackboard->close(cluster_pos_if_);
  blackboard->close(switch_if_);

  finput_.reset();
  fclusters_.reset();
}

void
LaserClusterThread::loop()
{
  TIMETRACK_START(ttc_full_loop_);

  ++loop_count_;

  TIMETRACK_START(ttc_msgproc_);

  while (! switch_if_->msgq_empty()) {
    if (SwitchInterface::EnableSwitchMessage *msg =
        switch_if_->msgq_first_safe(msg))
    {
      switch_if_->set_enabled(true);
      switch_if_->write();
    } else if (SwitchInterface::DisableSwitchMessage *msg =
               switch_if_->msgq_first_safe(msg))
    {
      switch_if_->set_enabled(false);
      switch_if_->write();
    }

    switch_if_->msgq_pop();
  }

  if (! switch_if_->is_enabled()) {
    TimeWait::wait(250000);
    return;
  }

  TIMETRACK_INTER(ttc_msgproc_, ttc_extract_lines_);

  CloudPtr noline_cloud(new Cloud());

  //pcl::search::KdTree<PointType> kdtree;

  if (input_->points.size() <= 10) {
    // this can happen if run at startup. Since tabletop threads runs continuous
    // and not synchronized with main loop, but point cloud acquisition thread is
    // synchronized, we might start before any data has been read
    //logger->log_warn(name(), "Empty voxelized point cloud, omitting loop");
    TimeWait::wait(50000);
    return;
  }

  // copy
  *noline_cloud = *input_;

  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

  while (noline_cloud->points.size () > cfg_cluster_min_size_) {
    // Segment the largest planar component from the remaining cloud
    seg_.setInputCloud (noline_cloud);
    seg_.segment(*inliers, *coeff);
    if (inliers->indices.size () == 0) {
      // no line found
      break;
    }

    // check for a minimum number of expected inliers
    if ((double)inliers->indices.size()
	< (cfg_segm_inlier_quota_ * (double)input_->points.size()))
    {
      logger->log_warn(name(), "[L %u] no lines (%zu inliers, required %f)",
                       loop_count_, inliers->indices.size(),
                       (cfg_segm_inlier_quota_ * input_->points.size()));
      break;
    }

    // Remove the linear inliers, extract the rest
    CloudPtr cloud_f(new Cloud());
    pcl::ExtractIndices<PointType> extract;
    extract.setNegative(true);
    extract.filter(*cloud_f);
    *noline_cloud = *cloud_f;
  }

  // What remains in the cloud are now potential clusters

  TIMETRACK_INTER(ttc_extract_lines_, ttc_clustering_);

  if (noline_cloud->points.size() > 0) {
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<PointType>::Ptr
      kdtree_cl(new pcl::search::KdTree<PointType>());
    kdtree_cl->setInputCloud(noline_cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointType> ec;
    ec.setClusterTolerance(cfg_cluster_tolerance_);
    ec.setMinClusterSize(cfg_cluster_min_size_);
    ec.setMaxClusterSize(cfg_cluster_max_size_);
    ec.setSearchMethod(kdtree_cl);
    ec.setInputCloud(noline_cloud);
    ec.extract(cluster_indices);

    logger->log_debug(name(), "Found %zu clusters", cluster_indices.size());

  } else {
    logger->log_info(name(), "Filter left no points for clustering");
  }

  //*clusters_ = *tmp_clusters;
  pcl_utils::copy_time(finput_, fclusters_);


  TIMETRACK_END(ttc_clustering_);
  TIMETRACK_END(ttc_full_loop_);

#ifdef USE_TIMETRACKER
  if (++tt_loopcount_ >= 5) {
    tt_loopcount_ = 0;
    tt_->print_to_stdout();
  }
#endif
}


void
LaserClusterThread::set_position(fawkes::Position3DInterface *iface,
                                    bool is_visible, const Eigen::Vector4f &centroid,
                                    const Eigen::Quaternionf &attitude)
{
  tf::Stamped<tf::Pose> baserel_pose;
  try{
    tf::Stamped<tf::Pose>
      spose(tf::Pose(tf::Quaternion(attitude.x(), attitude.y(), attitude.z(), attitude.w()),
                     tf::Vector3(centroid[0], centroid[1], centroid[2])),
            fawkes::Time(0, 0), input_->header.frame_id);
    tf_listener->transform_pose(cfg_result_frame_, spose, baserel_pose);
    iface->set_frame(cfg_result_frame_.c_str());
  } catch (tf::TransformException &e) {
    is_visible = false;
  }

  int visibility_history = iface->visibility_history();
  if (is_visible) {
    if (visibility_history >= 0) {
      iface->set_visibility_history(visibility_history + 1);
    } else {
      iface->set_visibility_history(1);
    }
    tf::Vector3 &origin = baserel_pose.getOrigin();
    tf::Quaternion quat = baserel_pose.getRotation();
    double translation[3] = { origin.x(), origin.y(), origin.z() };
    double rotation[4] = { quat.x(), quat.y(), quat.z(), quat.w() };
    iface->set_translation(translation);
    iface->set_rotation(rotation);
  
  } else {
    if (visibility_history <= 0) {
      iface->set_visibility_history(visibility_history - 1);
    } else {
      iface->set_visibility_history(-1);
      double translation[3] = { 0, 0, 0 };
      double rotation[4] = { 0, 0, 0, 1 };
      iface->set_translation(translation);
      iface->set_rotation(rotation);
    }
  }
  iface->write();  
}
