/***************************************************************************
 *  object_tracking_thread.h - Object Tracking Plugin
 *
 *  Created: Wed Apr 16 17:54:31 2014
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

#ifndef __PLUGINS_PERCEPTION_OBJECT_TRACKING_THREAD_H_
#define __PLUGINS_PERCEPTION_OBJECT_TRACKING_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/configurable.h>
#include <aspect/logging.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/syncpoint_manager.h>

#include <interfaces/Position3DInterface.h>
#include <interfaces/SwitchInterface.h>
#include <interfaces/ObjectTrackingInterface.h>

#include <vector>
#include <map>

#include <Eigen/StdVector>

#include "../common/perception_common.h"

namespace fawkes {
  class Position3DInterface;
  class SyncPoint;
}

/** @class OldCentroid "tabletop_objects_thread.h"
 * This class is used to save old centroids in order to check for reappearance
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

class ObjectTrackingThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::LoggingAspect,
  public fawkes::ConfigurableAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::BlockedTimingAspect,
  public fawkes::SyncPointManagerAspect
{
 public:
  ObjectTrackingThread();
  virtual ~ObjectTrackingThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();

 private:
  typedef std::map<unsigned int, Eigen::Vector4f, std::less<unsigned int>,
      Eigen::aligned_allocator<std::pair<const unsigned int, Eigen::Vector4f>>>
      CentroidMap;
  typedef std::list<OldCentroid, Eigen::aligned_allocator<OldCentroid> > OldCentroidVector;

 private:
  void set_position(fawkes::Position3DInterface *iface,
                    bool is_visible,
                    const Eigen::Vector4f &centroid = Eigen::Vector4f(0, 0, 0, 0),
                    const Eigen::Quaternionf &rotation = Eigen::Quaternionf(1, 0, 0, 0));
  std::map<unsigned int, int> track_objects(const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> &new_centroids);
  int next_id();
  void delete_old_centroids(OldCentroidVector &centroids, unsigned int age);
  void delete_near_centroids(const CentroidMap &reference, OldCentroidVector &centroids,
      float min_distance);

 private:
  std::list<fawkes::Position3DInterface *> pos_ifs_in_;
  std::map<uint, fawkes::Position3DInterface *> pos_ifs_out_;
  fawkes::SwitchInterface *switch_if_;
  fawkes::ObjectTrackingInterface *track_if_;

  bool first_run_;
  std::list<unsigned int> free_ids_;
  /** List of free ids which should be used first */
  std::list<unsigned int> prio_free_ids_;
  CentroidMap centroids_;
  OldCentroidVector old_centroids_;
  std::string frame_id_;

  /* synchronization */
  fawkes::RefPtr<fawkes::SyncPoint> syncpoint_in_;
  fawkes::RefPtr<fawkes::SyncPoint> syncpoint_out_;

  /* configuration */
  uint cfg_centroid_max_age_;
  float cfg_centroid_max_distance_;
  float cfg_centroid_min_distance_;
  std::string cfg_ifs_in_;
  std::string cfg_ifs_out_;
  std::string cfg_syncpoint_in_;
  std::string cfg_syncpoint_out_;

};

#endif
