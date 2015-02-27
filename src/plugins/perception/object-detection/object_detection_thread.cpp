/***************************************************************************
 *  object_detection_thread.cpp - Object Detection Plugin
 *
 *  Created: Tue Apr 15 17:04:45 2014
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

#include "object_detection_thread.h"
#include "../common/cluster_colors.h"
#include "../common/perception_common.h"

#include <tf/types.h>
#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>

#include <libs/syncpoint/exceptions.h>

#ifdef USE_TIMETRACKER
#  include <utils/time/tracker.h>
#endif
#include <utils/time/tracker_macros.h>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/statistical_outlier_removal.h>


using namespace std;
using namespace fawkes;
using namespace fawkes::perception;

#define CFG_PREFIX "/perception/object-detection/"

/** @class ObjectDetectionThread "object_detection_thread.h"
 * Thread to detect objects in a pointcloud
 * @author Till Hofmann
 */

/** Constructor. */
ObjectDetectionThread::ObjectDetectionThread()
: Thread("ObjectDetectionThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
  TransformAspect(TransformAspect::ONLY_LISTENER),
  SyncPointAspect(SyncPoint::WAIT_FOR_ALL, "/perception/tabletop-detection", "/perception/object-detection")
{
}

ObjectDetectionThread::~ObjectDetectionThread()
{
}

void
ObjectDetectionThread::init()
{
  // allow concurrent execution of prepare_finalize and loop
  // this is necessary to interrupt syncpoint wait calls
  set_prepfin_conc_loop(true);

  cfg_cluster_tolerance_              = config->get_float(CFG_PREFIX"cluster_tolerance");
  cfg_cluster_min_size_               = config->get_uint(CFG_PREFIX"cluster_min_size");
  cfg_cluster_max_size_               = config->get_uint(CFG_PREFIX"cluster_max_size");
  cfg_result_frame_                   = config->get_string(CFG_PREFIX"result_frame");
  cfg_input_pointcloud_               = config->get_string(CFG_PREFIX"input_pointcloud");
  cfg_centroid_max_height_            = config->get_float(CFG_PREFIX"centroid_max_height");
  cfg_centroid_min_distance_to_base_  = config->get_float(CFG_PREFIX"centroid_min_distance_to_base");
  cfg_syncpoint_in_                   = config->get_string(CFG_PREFIX"syncpoint_in");
  cfg_syncpoint_out_                  = config->get_string(CFG_PREFIX"syncpoint_out");

  cfg_verbose_output_ = true;
  try {
    cfg_verbose_output_ = config->get_bool(CFG_PREFIX"verbose_output");
  } catch (const Exception &e) {
    // ignored, use default
  }

  if (pcl_manager->exists_pointcloud<PointType>(cfg_input_pointcloud_.c_str())) {
     finput_ = pcl_manager->get_pointcloud<PointType>(cfg_input_pointcloud_.c_str());
     input_ = pcl_utils::cloudptr_from_refptr(finput_);
  } else {
    throw Exception("Point cloud '%s' does not exist or not XYZ or XYZ/RGB PCL",
        cfg_input_pointcloud_.c_str());
  }

  try {
    double rotation[4] = {0., 0., 0., 1.};
    pos_ifs_.clear();
    pos_ifs_.resize(MAX_CENTROIDS, NULL);
    for (unsigned int i = 0; i < MAX_CENTROIDS; ++i) {
      char *tmp;
      if (asprintf(&tmp, "Tabletop Object %u", i + 1) != -1) {
        // Copy to get memory freed on exception
        std::string id = tmp;
        free(tmp);
        Position3DInterface *iface =
            blackboard->open_for_writing<Position3DInterface>(id.c_str());
        pos_ifs_[i] = iface;
        iface->set_rotation(rotation);
        iface->write();
      }
    }

    switch_if_ = blackboard->open_for_writing<SwitchInterface>("object-detection");
    switch_if_->set_enabled(true);
    switch_if_->write();

  } catch (Exception &e) {
    for (unsigned int i = 0; i < MAX_CENTROIDS; ++i) {
      if (pos_ifs_[i]) {
        blackboard->close(pos_ifs_[i]);
      }
    }
    blackboard->close(switch_if_);
    throw;
  }

  char *tmp_name;
  fawkes::RefPtr<pcl::PointCloud<ColorPointType> > f_tmp_cloud;
  obj_clusters_.clear();
  f_obj_clusters_.clear();
  pcl::PointCloud<ColorPointType>::Ptr tmp_cloud;
  for (int i = 0; i < MAX_CENTROIDS; i++) {
    f_tmp_cloud = new pcl::PointCloud<ColorPointType>();
    f_tmp_cloud->header.frame_id = input_->header.frame_id;
    f_tmp_cloud->is_dense = false;
    std::string obj_id;
    if (asprintf(&tmp_name, "obj_cluster_%u", i) != -1) {
      obj_id = tmp_name;
      free(tmp_name);
    }
    pcl_manager->add_pointcloud<ColorPointType> (obj_id.c_str(), f_tmp_cloud);
    f_obj_clusters_.push_back(f_tmp_cloud);
    tmp_cloud = pcl_utils::cloudptr_from_refptr(f_tmp_cloud);
    obj_clusters_.push_back(tmp_cloud);
  }

  // open table position interface
  // needed for height filter
  try {
    table_pos_if_ = blackboard->open_for_reading<Position3DInterface>("Tabletop");
  } catch (Exception &e) {
    logger->log_error(name(), "Tabletop position interface doesn't exist. Did you load the tabletop-detection plugin?");
    throw;
  }

  centroids_.clear();

#ifdef USE_TIMETRACKER
  tt_ = new TimeTracker();
  tt_loopcount_ = 0;
  ttc_full_loop_      = tt_->add_class("Full Loop");
  ttc_syncpoint_wait_ = tt_->add_class("SyncPoint Wait Call");
  ttc_obj_extraction_ = tt_->add_class("Object Extraction");
#endif

}

void
ObjectDetectionThread::finalize()
{
  input_.reset();
  for (vector<ColorCloudPtr>::iterator it = obj_clusters_.begin();
      it != obj_clusters_.end(); it++) {
    it->reset();
  }

  char *tmp_name;
  for (unsigned int i = 0; i < MAX_CENTROIDS; i++) {
    std::string obj_id;
    if (asprintf(&tmp_name, "obj_cluster_%u", i) != -1) {
      obj_id = tmp_name;
      free(tmp_name);
    }
    pcl_manager->remove_pointcloud(obj_id.c_str());
  }

  blackboard->close(table_pos_if_);
  blackboard->close(switch_if_);

  for (PosIfsVector::iterator it = pos_ifs_.begin(); it != pos_ifs_.end(); it++) {
    blackboard->close(*it);
  }

  finput_.reset();
  for (vector<RefPtr<ColorCloud> >::iterator it = f_obj_clusters_.begin();
      it != f_obj_clusters_.end(); it++) {
    it->reset();
  }

}

void
ObjectDetectionThread::loop()
{

  TIMETRACK_START(ttc_full_loop_);

  while (! switch_if_->msgq_empty()) {
    if (SwitchInterface::EnableSwitchMessage *msg =
        switch_if_->msgq_first_safe(msg))
    {
      switch_if_->set_enabled(true);
      switch_if_->write();
    } else if (SwitchInterface::DisableSwitchMessage *msg =
               switch_if_->msgq_first_safe(msg))
    {
      for (PosIfsVector::iterator it = pos_ifs_.begin(); it != pos_ifs_.end(); it++) {
        (*it)->set_visibility_history(0);
        (*it)->write();
      }
      switch_if_->set_enabled(false);
      switch_if_->write();
    }

    switch_if_->msgq_pop();
  }

  if (! switch_if_->is_enabled()) {
    TIMETRACK_ABORT(ttc_full_loop_);
    return;
  }

  table_pos_if_->read();
  if (table_pos_if_->visibility_history() < 0) {
    logger->log_debug(name(), "No tabletop. Aborting object detection.");
    return;
  }
  CloudPtr cloud_objs_(new Cloud(*input_));
  cloud_objs_->header.frame_id = input_->header.frame_id;
  ColorCloudPtr tmp_clusters(new ColorCloud());

  unsigned int object_count = 0;
    if (cloud_objs_->points.size() > 0) {
      //TODO: perform statistical outlier removal at this point before clustering.
      //Outlier removal
      pcl::StatisticalOutlierRemoval<PointType> sor;
      sor.setInputCloud(cloud_objs_);
      sor.setMeanK(5);
      sor.setStddevMulThresh(0.2);
      sor.filter(*cloud_objs_);
    }
    //OBJECTS
    std::vector<ColorCloudPtr> tmp_obj_clusters(MAX_CENTROIDS);
    object_count = cluster_objects(cloud_objs_, tmp_clusters, tmp_obj_clusters);
    if (object_count == 0) {
      if (cfg_verbose_output_)
        logger->log_info(name(), "No clustered points found");
    }

    // set all pos_ifs not in centroids_ to 'not visible'
    for (unsigned int i = 0; i < pos_ifs_.size(); i++) {
      if (!centroids_.count(i)) {
        set_position(pos_ifs_[i], false);
      }
    }

    // set positions of all visible centroids
    for (CentroidMap::iterator it = centroids_.begin(); it != centroids_.end(); it++) {
      set_position(pos_ifs_[it->first], true, it->second, Eigen::Quaternionf(1, 0, 0, 0), "/base_link");
    }

    for (unsigned int i = 0; i < f_obj_clusters_.size(); i++) {
      if (centroids_.count(i)) {
        *obj_clusters_[i] = *tmp_obj_clusters[i];
        obj_clusters_[i]->header.frame_id = tmp_obj_clusters[i]->header.frame_id;
      }
      else {
        obj_clusters_[i]->clear();
        // add point to force update
        // TODO find proper way to update an empty cloud
  //      obj_clusters_[i]->push_back(ColorPointType());
      }
      pcl_utils::copy_time(input_, f_obj_clusters_[i]);
    }

    TIMETRACK_END(ttc_full_loop_);
#ifdef USE_TIMETRACKER
  if (++tt_loopcount_ >= 5) {
    tt_loopcount_ = 0;
    tt_->print_to_stdout();
  }
#endif
}

unsigned int
ObjectDetectionThread::cluster_objects(CloudConstPtr input_cloud,
    ColorCloudPtr tmp_clusters,
    std::vector<ColorCloudPtr> &tmp_obj_clusters) {
  unsigned int object_count = 0;
  std::vector<pcl::PointIndices> cluster_indices = extract_object_clusters(input_cloud);
  std::vector<pcl::PointIndices>::const_iterator it;
  unsigned int num_points = 0;
  for (it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    num_points += it->indices.size();

  CentroidMap tmp_centroids;

  if (num_points > 0) {
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> new_centroids(MAX_CENTROIDS);

    unsigned int centroid_i = 0;
    for (it = cluster_indices.begin();
        it != cluster_indices.end() && centroid_i < MAX_CENTROIDS;
        ++it, ++centroid_i)
    {
      //Centroids in cam frame:
      //pcl::compute3DCentroid(*cloud_objs_, it->indices, centroids[centroid_i]);

      // TODO fix this; we only want to copy the cluster, the color is incorrect
      ColorCloudPtr single_cluster =
          colorize_cluster(input_cloud, it->indices, cluster_colors[centroid_i]);
      single_cluster->header.frame_id = input_cloud->header.frame_id;
      single_cluster->width = it->indices.size();
      single_cluster->height = 1;

      ColorCloudPtr obj_in_base_frame(new ColorCloud());
      obj_in_base_frame->header.frame_id = "/base_link";
      obj_in_base_frame->width = it->indices.size();
      obj_in_base_frame->height = 1;
      obj_in_base_frame->points.resize(it->indices.size());

      // don't add cluster here since the id is wrong
      //*obj_clusters_[obj_i++] = *single_cluster;

pcl_utils::transform_pointcloud("/base_link", *single_cluster,
        *obj_in_base_frame, *tf_listener);

      pcl::compute3DCentroid(*obj_in_base_frame, new_centroids[centroid_i]);

//      if (cfg_cylinder_fitting_) {
//        new_centroids[centroid_i] = fit_cylinder(obj_in_base_frame,
//            new_centroids[centroid_i], centroid_i);
//      }


    }
    object_count = centroid_i;
    new_centroids.resize(object_count);


    std::map<uint, int> assigned_ids;
//    if (cfg_track_objects_) {
//      assigned_ids = track_objects(new_centroids);
//    }
//    else { //! cfg_track_objects_
      for (unsigned int i = 0; i < new_centroids.size(); i++) {
        assigned_ids[i] = i;
      }
//    }

    // reassign IDs
    for (uint i = 0; i < new_centroids.size(); i++) {
      int assigned_id;
      try {
        assigned_id = assigned_ids.at(i);
      }
      catch (const std::out_of_range& e) {
        logger->log_error(name(), "Object %d was not assigned", i);
        // drop centroid
        assigned_id = -1;
      }
      if (assigned_id == -1)
        continue;
      tmp_centroids[assigned_id] = new_centroids[i];
      ColorCloudPtr colorized_cluster = colorize_cluster(input_cloud,
          cluster_indices[i].indices,
          cluster_colors[assigned_id % MAX_CENTROIDS]);
      colorized_cluster->header.frame_id = input_cloud->header.frame_id;
      *tmp_clusters += *colorized_cluster;
      tmp_obj_clusters[assigned_id] = colorized_cluster;
      tmp_obj_clusters[assigned_id]->header.frame_id = colorized_cluster->header.frame_id;
    }


    // remove all centroids too high above the table
    double *translation(table_pos_if_->translation());
    Eigen::Vector4f table_centroid(translation[0], translation[1], translation[2], 0.f);
    delete_high_centroids(table_centroid, tmp_centroids);
    delete_centroids_close_to_base(tmp_centroids);

//    if (object_count > 0)
//      first_run_ = false;
  }
  else {
    if (cfg_verbose_output_)
      logger->log_info(name(), "No clustered points found");
//    // save all centroids to old centroids
//    for (CentroidMap::iterator it = centroids_.begin(); it != centroids_.end(); it++) {
//      old_centroids_.push_back(OldCentroid(it->first, it->second));
//    }
  }
  centroids_ = tmp_centroids;
  return object_count;
}

void
ObjectDetectionThread::set_position(fawkes::Position3DInterface *iface,
                                    bool is_visible,
                                    const Eigen::Vector4f &centroid,
                                    const Eigen::Quaternionf &attitude,
                                    string source_frame)
{
  if (source_frame == "") {
    source_frame = input_->header.frame_id;
  }
  tf::Stamped<tf::Pose> baserel_pose;
  try{
    tf::Stamped<tf::Pose> spose = centroid_to_pose(centroid, attitude, source_frame);
    tf_listener->transform_pose(cfg_result_frame_, spose, baserel_pose);
    iface->set_frame(cfg_result_frame_.c_str());
  } catch (Exception &e) {
    set_pos_interface(iface, false);
  }

  set_pos_interface(iface, is_visible, baserel_pose);
}

std::vector<pcl::PointIndices>
ObjectDetectionThread::extract_object_clusters(CloudConstPtr input) {
  TIMETRACK_START(ttc_obj_extraction_);
  std::vector<pcl::PointIndices> cluster_indices;
  if (input->empty()) {
    TIMETRACK_ABORT(ttc_obj_extraction_);
    return cluster_indices;
  }
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointType>::Ptr
  kdtree_cl(new pcl::search::KdTree<PointType>());
  kdtree_cl->setInputCloud(input);
  pcl::EuclideanClusterExtraction<PointType> ec;
  ec.setClusterTolerance(cfg_cluster_tolerance_);
  ec.setMinClusterSize(cfg_cluster_min_size_);
  ec.setMaxClusterSize(cfg_cluster_max_size_);
  ec.setSearchMethod(kdtree_cl);
  ec.setInputCloud(input);
  ec.extract(cluster_indices);

  //logger->log_debug(name(), "Found %zu clusters", cluster_indices.size());
  TIMETRACK_END(ttc_obj_extraction_);

  return cluster_indices;

}

void
ObjectDetectionThread::delete_high_centroids(Eigen::Vector4f table_centroid,
  CentroidMap &centroids) {
  tf::Stamped<tf::Point> sp_baserel_table;
  tf::Stamped<tf::Point> sp_table(
      tf::Point(table_centroid[0], table_centroid[1], table_centroid[2]),
      fawkes::Time(0, 0), input_->header.frame_id);
  try {
    tf_listener->transform_point("/base_link", sp_table, sp_baserel_table);
    for (CentroidMap::iterator it = centroids.begin(); it != centroids.end();) {
      try {
        tf::Stamped<tf::Point> sp_baserel_centroid(
            tf::Point(it->second[0], it->second[1], it->second[2]),
            fawkes::Time(0, 0), "/base_link");
        float d = sp_baserel_centroid.z() - sp_baserel_table.z();
        if (d > cfg_centroid_max_height_) {
          //logger->log_debug(name(), "remove centroid %u, too high (d=%f)", it->first, d);
//          free_ids_.push_back(it->first);
          centroids.erase(it++);
        } else
          it++;
      } catch (tf::TransformException &e) {
        // simply keep the centroid if we can't transform it
        it++;
      }
    }
  } catch (tf::TransformException &e) {
    // keep all centroids if transformation of the table fails
  }
}

void
ObjectDetectionThread::delete_centroids_close_to_base (CentroidMap &centroids) {
  for (CentroidMap::iterator it = centroids.begin(); it != centroids.end();) {
    try {
      tf::Stamped<tf::Point> sp_baserel_centroid(
          tf::Point(it->second[0], it->second[1], it->second[2]),
          fawkes::Time(0, 0), "/base_link");
      float d = sp_baserel_centroid.x();
      if (d < cfg_centroid_min_distance_to_base_) {
        logger->log_debug(name(), "remove centroid %u, too close to base (d=%f)", it->first, d);
        centroids.erase(it++);
      } else
        it++;
    } catch (tf::TransformException &e) {
      // simply keep the centroid if we can't transform it
      it++;
    }
  }
}
