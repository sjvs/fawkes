/***************************************************************************
 *  object_fitting_thread.cpp - Object Fitting
 *
 *  Created: Mon Apr 28 15:41:22 2014
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

#include "object_fitting_thread.h"

#include <pcl_utils/utils.h>
#include <utils/time/wait.h>

#include <interfaces/SwitchInterface.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/common/common.h>

#include <fnmatch.h>

using namespace std;
using namespace fawkes;
using namespace fawkes::perception;

#define CFG_PREFIX "/perception/object-fitting/"

/** @class ObjectFittingThread "object_fitting_thread.h"
 * Thread to fit objects to known shapes
 * @author Till Hofmann
 */

/** Constructor. */
ObjectFittingThread::ObjectFittingThread()
  : Thread("ObjectFittingThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}

/** Destructor. */
ObjectFittingThread::~ObjectFittingThread()
{
}

void
ObjectFittingThread::init()
{

  cfg_baselink_frame_id_ = "/base_link";
  try {
    cfg_baselink_frame_id_ = config->get_string(CFG_PREFIX"base_link_frame_id");
  } catch (Exception &e) {
    // use default
  }

  cfg_verbose_output_ = false;
  try {
    cfg_verbose_output_ = config->get_bool(CFG_PREFIX"verbose");
  } catch (Exception &e) {
    // use default
  }

  cfg_pointclouds_          = config->get_string(CFG_PREFIX"input_pointclouds");
  cfg_output_prefix_        = config->get_string(CFG_PREFIX"output_prefix");
  cfg_use_colored_input_    = config->get_bool(CFG_PREFIX"use_colored_input");
  cfg_syncpoint_in_         = config->get_string(CFG_PREFIX"syncpoint_in");
  cfg_syncpoint_out_        = config->get_string(CFG_PREFIX"syncpoint_out");

  // find all pointclouds
  vector<string> pointclouds = pcl_manager->get_pointcloud_list();

  for (vector<string>::iterator it = pointclouds.begin(); it != pointclouds.end(); it++) {
    // TODO this pointcloud matching should happen in the pcl_manager
    if (!fnmatch(cfg_pointclouds_.c_str(), it->c_str(), 0)) {
      logger->log_debug(name(), "Opening point cloud '%s'", it->c_str());
      if (cfg_use_colored_input_) {
        if (pcl_manager->exists_pointcloud<ColorPointType>(it->c_str())) {
          // convert to XYZ cloud
          RefPtr<const pcl::PointCloud<ColorPointType> > colored_cloud =
              pcl_manager->get_pointcloud<ColorPointType>(it->c_str());
          fcolored_input_.push_back(colored_cloud);
          colored_input_.push_back(pcl_utils::cloudptr_from_refptr(colored_cloud));
          RefPtr<const pcl::PointCloud<PointType> > fconverted_cloud;
          CloudConstPtr converted_cloud = pcl_utils::cloudptr_from_refptr(fconverted_cloud);
          converted_cloud.reset(new Cloud());
          finput_.push_back(fconverted_cloud);
          input_.push_back(converted_cloud);
        } else {
          throw Exception("Point cloud '%s' is not a XYZ/RGB point cloud. Is use_colored_output set correctly?", it->c_str());
        }
      } else {
        if (pcl_manager->exists_pointcloud<PointType>(it->c_str())) {
          RefPtr<const pcl::PointCloud<PointType> > cloud =
              pcl_manager->get_pointcloud<PointType>(it->c_str());
          finput_.push_back(cloud);
          input_.push_back(pcl_utils::cloudptr_from_refptr(cloud));
        } else {
          throw Exception("Point cloud '%s' is not a XYZ point cloud. Is use_colored_output set correctly?", it->c_str());
        }
      }
    }
  }

  try {
    pos_ifs_.resize(input_.size());
    for (uint i = 0; i < input_.size(); i++) {
      char *tmp;
      string pattern = cfg_output_prefix_ + " %u";
      if (asprintf(&tmp, pattern.c_str(), i + 1)) {
        string id = tmp;
        free(tmp);
        Position3DInterface *iface = blackboard->open_for_writing<Position3DInterface>(id.c_str());
        set_pos_interface(iface, false);
        pos_ifs_[i] = iface;
      }
    }

    switch_if_ = blackboard->open_for_writing<SwitchInterface>("object-fitting");
    switch_if_->set_enabled(true);
    switch_if_->write();

  } catch (Exception &e) {
    // close all interfaces
    for (vector<Position3DInterface *>::iterator it = pos_ifs_.begin(); it != pos_ifs_.end(); it++) {
      blackboard->close(*it);
    }
    blackboard->close(switch_if_);
    throw;
  }

  //TODO must do initialization better (look-up table for known objects)

  obj_likelihoods_.clear();
  known_obj_dimensions_.clear();
  obj_shape_confidence_.clear();
  cylinder_params_.clear();
  best_obj_guess_.clear();

  known_obj_dimensions_.resize(3);

  //Green cup
  known_obj_dimensions_[0][0] = 0.07;
  known_obj_dimensions_[0][1] = 0.07;
  known_obj_dimensions_[0][2] = 0.104;
  //Red cup
  known_obj_dimensions_[1][0] = 0.088;
  known_obj_dimensions_[1][1] = 0.088;
  known_obj_dimensions_[1][2] = 0.155;
  //White cylinder
  known_obj_dimensions_[2][0] = 0.106;
  known_obj_dimensions_[2][1] = 0.106;
  known_obj_dimensions_[2][2] = 0.277;

  std::vector<double> init_likelihoods;
  init_likelihoods.resize(known_obj_dimensions_.size() + 1, 0.0);
  // TODO obj_likelihoods_ initialization

  syncpoint_in_ = syncpoint_manager->get_syncpoint(name(), cfg_syncpoint_in_.c_str());
  syncpoint_out_ = syncpoint_manager->get_syncpoint(name(), cfg_syncpoint_out_.c_str());

}

void
ObjectFittingThread::finalize()
{
  for (vector<Position3DInterface *>::iterator it = pos_ifs_.begin(); it != pos_ifs_.end(); it++) {
    blackboard->close(*it);
  }

  blackboard->close(switch_if_);

  syncpoint_manager->release_syncpoint(name(), syncpoint_in_);
  syncpoint_manager->release_syncpoint(name(), syncpoint_out_);
}

void
ObjectFittingThread::loop()
{

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

  syncpoint_in_->wait(name());

  if (cfg_use_colored_input_) {
    for (uint i = 0; i < input_.size(); i++) {
      CloudPtr converted_input;
      converted_input.reset(new Cloud());
      convert_colored_input(colored_input_[i], converted_input);
      input_[i] = converted_input;
    }
  }

  std::vector<double> init_likelihoods;
  init_likelihoods.resize(input_.size() + 1, 0.0);
  cylinder_params_.resize(input_.size());
  for (uint i = 0; i < input_.size(); i++)
    obj_likelihoods_[i] = init_likelihoods;
  for (uint i = 0; i < input_.size(); i++) {
    if (input_[i]->size() == 0) {
      set_pos_interface(pos_ifs_[i], false);
      continue;
    }
    CloudPtr obj_in_base_frame(new Cloud());
    obj_in_base_frame->header.frame_id = cfg_baselink_frame_id_;
    obj_in_base_frame->width = input_[i]->size();
    obj_in_base_frame->height = 1;
    obj_in_base_frame->points.resize(input_[i]->size());
    try {
      pcl_utils::transform_pointcloud(cfg_baselink_frame_id_, *input_[i],
        *obj_in_base_frame, *tf_listener);
      Eigen::Vector4f centroid = fit_cylinder(obj_in_base_frame, i + 1);
      set_position(pos_ifs_[i], true, centroid);
    } catch (Exception &e) {
//      logger->log_warn(name(), "Cylinder fitting for object %u failed: %s", i + 1, e.what());
      set_pos_interface(pos_ifs_[i], false);
    }
  }

  syncpoint_out_->emit(name());
}

Eigen::Vector4f
ObjectFittingThread::fit_cylinder(
  CloudConstPtr obj_in_base_frame, uint const &centroid_i)
{
  Eigen::Vector4f centroid;
  PointType pnt_min, pnt_max;
  Eigen::Vector3f obj_dim;
  std::vector < Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>
      > obj_size_scores;
  pcl::getMinMax3D(*obj_in_base_frame, pnt_min, pnt_max);
  obj_dim[0] = fabs(pnt_max.x - pnt_min.x);
  obj_dim[1] = fabs(pnt_max.y - pnt_min.y);
  obj_dim[2] = fabs(pnt_max.z - pnt_min.z);
  compute_bounding_box_scores(obj_dim, obj_size_scores);

  if (cfg_verbose_output_) {
    logger->log_debug(name(), "*******************Processing obj_%u******************",
    centroid_i);
    logger->log_debug(name(), "Computed object dimensions: %f %f %f", obj_dim[0],
        obj_dim[1], obj_dim[2]);
    logger->log_debug(name(), "Size similarity to known objects:");
  }
  obj_likelihoods_[centroid_i].resize(known_obj_dimensions_.size());
  for (uint os = 0; os < known_obj_dimensions_.size(); os++) {
    if (cfg_verbose_output_) {
      logger->log_debug(name(), "** Cup %i: %f in x, %f in y, %f in z.", os,
          obj_size_scores[os][0], obj_size_scores[os][1], obj_size_scores[os][2]);
    }
    obj_likelihoods_[centroid_i][os] = obj_size_scores[os][0] *
        obj_size_scores[os][1] * obj_size_scores[os][2];
  }

  // Fit cylinder
  pcl::NormalEstimation < PointType, pcl::Normal > ne;
  pcl::SACSegmentationFromNormals < PointType, pcl::Normal > seg;
  pcl::ExtractIndices < PointType > extract;
  pcl::ExtractIndices < pcl::Normal > extract_normals;
  pcl::PointCloud<pcl::Normal>::Ptr obj_normals(
      new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<PointType>::Ptr tree_cyl(
      new pcl::search::KdTree<PointType>());
  pcl::ModelCoefficients::Ptr coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_cylinder(new pcl::PointIndices);

  // Estimate point normals
  ne.setSearchMethod(tree_cyl);
  ne.setInputCloud(obj_in_base_frame);
  ne.setKSearch(10);
  ne.compute(*obj_normals);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_CYLINDER);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight(0.1);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.07);
  seg.setRadiusLimits(0, 0.12);

  seg.setInputCloud(obj_in_base_frame);
  seg.setInputNormals(obj_normals);

  // Obtain the cylinder inliers and coefficients

  seg.segment(*inliers_cylinder, *coefficients_cylinder);
  //Getting max and min z values from cylinder inliers.
  extract.setInputCloud(obj_in_base_frame);
  extract.setIndices(inliers_cylinder);
  extract.setNegative(false);
  pcl::PointCloud<PointType>::Ptr cloud_cylinder_baserel(
      new pcl::PointCloud<PointType>());
  extract.filter(*cloud_cylinder_baserel);

  cylinder_params_[centroid_i] = Eigen::Vector4f();
  cylinder_params_[centroid_i][0] = 0;
  cylinder_params_[centroid_i][1] = 0;
  if (cloud_cylinder_baserel->points.empty()) {
    logger->log_debug(name(), "No cylinder inliers!!");
    obj_shape_confidence_[centroid_i] = 0.0;
    throw Exception("No cylinder inliers.");
  }
  else {

    if (!tf_listener->frame_exists(cloud_cylinder_baserel->header.frame_id)) {
      throw Exception("Frame '%s' doesn't exist", cloud_cylinder_baserel->header.frame_id.c_str());
    }

    obj_shape_confidence_[centroid_i] = (double) (cloud_cylinder_baserel->points
      .size()) / (obj_in_base_frame->points.size() * 1.0);
    if (cfg_verbose_output_) {
      logger->log_debug(name(), "Cylinder fit confidence = %zu/%zu = %f",
          cloud_cylinder_baserel->points.size(), obj_in_base_frame->points.size(),
          obj_shape_confidence_[centroid_i]);
    }

    PointType pnt_min;
    PointType pnt_max;
    pcl::getMinMax3D(*cloud_cylinder_baserel, pnt_min, pnt_max);
    if (cfg_verbose_output_) {
      logger->log_debug(name(),
          "Cylinder height according to cylinder inliers: %f",
          pnt_max.z - pnt_min.z);
      logger->log_debug(name(), "Cylinder height according to bounding box: %f",
          obj_dim[2]);
      logger->log_debug(name(),
          "Cylinder radius according to cylinder fitting: %f",
          (*coefficients_cylinder).values[6]);
      logger->log_debug(name(),
          "Cylinder radius according to bounding box y: %f", obj_dim[1] / 2);
    }
    //Cylinder radius:
    //cylinder_params_[centroid_i][0] = (*coefficients_cylinder).values[6];
    cylinder_params_[centroid_i][0] = obj_dim[1] / 2;
    //Cylinder height:
    //cylinder_params_[centroid_i][1] = (pnt_max->z - pnt_min->z);
    cylinder_params_[centroid_i][1] = obj_dim[2];

    //cylinder_params_[centroid_i][2] = table_inclination_;

    //Overriding computed centroids with estimated cylinder center:
    centroid[0] = pnt_min.x + 0.5 * (pnt_max.x - pnt_min.x);
    centroid[1] = pnt_min.y + 0.5 * (pnt_max.y - pnt_min.y);
    centroid[2] = pnt_min.z + 0.5 * (pnt_max.z - pnt_min.z);
  }

  signed int detected_obj_id = -1;
  double best_confidence = 0.0;
  if (cfg_verbose_output_) {
    logger->log_debug(name(), "Shape similarity = %f",
        obj_shape_confidence_[centroid_i]);
  }
  for (uint os = 0; os < known_obj_dimensions_.size(); os++) {
    obj_likelihoods_[centroid_i][os] =
      (0.6 * obj_likelihoods_[centroid_i][os])
          + (0.4 * obj_shape_confidence_[centroid_i]);
    if (cfg_verbose_output_) {
      logger->log_debug(name(), "** Similarity to known cup %i:", os);
      logger->log_debug(name(), "Size similarity  = %f",
          obj_likelihoods_[centroid_i][os]);
      logger->log_debug(name(), "Overall similarity = %f",
          obj_likelihoods_[centroid_i][os]);
    }
    if (obj_likelihoods_[centroid_i][os] > best_confidence) {
      best_confidence = obj_likelihoods_[centroid_i][os];
      detected_obj_id = os;
    }
  }
  if (cfg_verbose_output_) {
    logger->log_debug(name(),
        "********************Object Result********************");
  }
  if (best_confidence > 0.6) {
    best_obj_guess_[centroid_i] = detected_obj_id;

    if (cfg_verbose_output_) {
      logger->log_debug(name(),
          "MATCH FOUND!! -------------------------> Cup number %i",
          detected_obj_id);
    }
  }
  else {
    best_obj_guess_[centroid_i] = -1;
    if (cfg_verbose_output_) {
      logger->log_debug(name(), "No match found.");
    }
  }
  if (cfg_verbose_output_) {
    logger->log_debug(name(),
        "*****************************************************");
  }

  return centroid;
}

void
ObjectFittingThread::set_position(fawkes::Position3DInterface *iface,
                                    bool is_visible,
                                    const Eigen::Vector4f &centroid)
{
  Eigen::Quaternionf attitude(1, 0, 0, 0);
  tf::Stamped<tf::Pose> spose = centroid_to_pose(centroid, attitude, cfg_baselink_frame_id_);
  iface->set_frame(cfg_baselink_frame_id_.c_str());

  set_pos_interface(iface, is_visible, spose);
}

bool
ObjectFittingThread::compute_bounding_box_scores(
    Eigen::Vector3f& cluster_dim,
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> >& scores)
{

  scores.resize(known_obj_dimensions_.size());

  for (uint i = 0; i < known_obj_dimensions_.size(); i++) {
    scores[i][0] = compute_similarity(cluster_dim[0],
        known_obj_dimensions_[i][0]);
    scores[i][1] = compute_similarity(cluster_dim[1],
        known_obj_dimensions_[i][1]);
    scores[i][2] = compute_similarity(cluster_dim[2],
        known_obj_dimensions_[i][2]);
  }
  return true;
}

double
ObjectFittingThread::compute_similarity(double d1, double d2)
{
  return exp(-50.0 * ((d1 - d2) * (d1 - d2)));
}
