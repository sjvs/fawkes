/***************************************************************************
 *  object_detection_thread.h - Object Detection Plugin
 *
 *  Created: Tue Apr 15 17:03:14 2014
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

#ifndef __PLUGINS_PERCEPTION_OBJECT_DETECTION_THREAD_H_
#define __PLUGINS_PERCEPTION_OBJECT_DETECTION_THREAD_H_

// must be first for reliable ROS detection
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/tf.h>
#include <aspect/pointcloud.h>
#include <aspect/syncpoint.h>

#include <Eigen/StdVector>

#include <vector>
#include <map>
#include <list>
#include <string>

#include "../common/perception_common.h"

namespace fawkes {
  class Position3DInterface;
  class SwitchInterface;
#ifdef USE_TIMETRACKER
  class TimeTracker;
#endif
}

/** @class OldCentroid "object_detection_thread.h"
 * This class is used to save old centroids in order to check for reappearance
 * @author Till Hofmann
 */
class OldCentroid {
public:
  /**
   * Constructor
   * @param id The ID which the centroid was assigned to
   * @param centroid The position of the centroid
   */
  OldCentroid(const unsigned int &id, const Eigen::Vector4f &centroid)
  : id_(id), age_(0), centroid_(centroid) { }
  /** Copy constructor
   * @param other The other OldCentroid */
  OldCentroid(const OldCentroid &other)
  : id_(other.getId()), age_(other.getAge()), centroid_(other.getCentroid()) { }
  virtual ~OldCentroid() { }
  // any class with Eigen::Vector4f needs a custom new operator
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  /** get the ID of the centroid
   * @return the ID of the centroid */
  unsigned int getId() const { return id_; }
  /** get the position of the centroid
   * @return a reference to the centroid */
  const Eigen::Vector4f& getCentroid() const { return centroid_; }
  /** get the age of the centroid
   * @return the number of loops the centroids has been invisible
   */
  unsigned int getAge() const { return age_; }
  /** increment the age of the centroid */
  void age() { age_++; }

protected:
  /** The ID of the centroid */
  unsigned int id_;
  /** The number of loops the centroid has been invisible for */
  unsigned int age_;
  /** The position of centroid */
  Eigen::Vector4f centroid_;
};


class ObjectDetectionThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::TransformAspect,
  public fawkes::PointCloudAspect,
  public fawkes::SyncPointAspect
{
 public:
  ObjectDetectionThread();
  virtual ~ObjectDetectionThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  typedef std::map<unsigned int, Eigen::Vector4f, std::less<unsigned int>,
      Eigen::aligned_allocator<std::pair<const unsigned int, Eigen::Vector4f> > > CentroidMap;
  typedef std::list<OldCentroid, Eigen::aligned_allocator<OldCentroid> > OldCentroidVector;
  typedef std::vector<fawkes::Position3DInterface *> PosIfsVector;

  /** Stub to see name in backtrace for easier debugging. @see Thread::run() */
  protected: virtual void run() {Thread::run();}

 private:
  void set_position(fawkes::Position3DInterface *iface,
                    bool is_visible,
                    const Eigen::Vector4f &centroid = Eigen::Vector4f(0, 0, 0, 0),
                    const Eigen::Quaternionf &rotation = Eigen::Quaternionf(1, 0, 0, 0),
                    std::string source_frame = "");
  std::vector<pcl::PointIndices> extract_object_clusters(fawkes::perception::CloudConstPtr input);
  void delete_high_centroids(Eigen::Vector4f table_centroid,
    CentroidMap &centroids);
  void delete_centroids_close_to_base(CentroidMap &centroids);
  unsigned int cluster_objects(fawkes::perception::CloudConstPtr input,
    fawkes::perception::ColorCloudPtr tmp_clusters,
    std::vector<fawkes::perception::ColorCloudPtr> &tmp_obj_clusters);


 private:
  fawkes::RefPtr<const fawkes::perception::Cloud> finput_;
  fawkes::perception::CloudConstPtr input_;

  std::vector<fawkes::RefPtr<fawkes::perception::ColorCloud> > f_obj_clusters_;
  std::vector<fawkes::perception::ColorCloudPtr> obj_clusters_;

  CentroidMap centroids_;

  PosIfsVector pos_ifs_;
  fawkes::Position3DInterface *table_pos_if_;

  fawkes::SwitchInterface *switch_if_;

  /* configuration */
  float cfg_cluster_tolerance_;
  unsigned int cfg_cluster_min_size_;
  unsigned int cfg_cluster_max_size_;
  std::string cfg_result_frame_;
  std::string cfg_input_pointcloud_;
  float cfg_centroid_max_height_;
  std::string cfg_syncpoint_in_;
  std::string cfg_syncpoint_out_;
  bool cfg_verbose_output_;
  float cfg_centroid_min_distance_to_base_;

#ifdef USE_TIMETRACKER
  fawkes::TimeTracker  *tt_;
  unsigned int tt_loopcount_;
  unsigned int ttc_full_loop_;
  unsigned int ttc_syncpoint_wait_;
  unsigned int ttc_obj_extraction_;
#endif
};

#endif
