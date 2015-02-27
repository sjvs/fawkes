/***************************************************************************
 *  object_tracking_thread.cpp - Object Tracking Plugin
 *
 *  Created: Wed Apr 23 12:58:07 2014
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

#include "object_tracking_thread.h"

#include <tf/types.h>

#include <utils/hungarian_method/hungarian.h>
#include <pcl/registration/distances.h>

#include <libs/syncpoint/exceptions.h>

using namespace std;
using namespace fawkes;
using namespace fawkes::perception;

#define CFG_PREFIX "/perception/object-tracking/"

/** @class ObjectTrackingThread "object_tracking_thread.h"
 * Main thread of object tracking plugin.
 * @author Till Hofmann
 */

/** Constructor. */
ObjectTrackingThread::ObjectTrackingThread()
  : Thread("ObjectTrackingThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
    SyncPointAspect(SyncPoint::WAIT_FOR_ALL, "/perception/object-detection", "/perception/object-tracking")
{
}

ObjectTrackingThread::~ObjectTrackingThread()
{
}

void
ObjectTrackingThread::init()
{
  // allow concurrent execution of prepare_finalize and loop
  // this is necessary to interrupt syncpoint wait calls
  set_prepfin_conc_loop(true);

  cfg_centroid_max_distance_ = config->get_float(CFG_PREFIX"centroid_max_distance");
  cfg_ifs_in_                = config->get_string(CFG_PREFIX"interfaces_in");
  cfg_ifs_out_               = config->get_string(CFG_PREFIX"interfaces_out");
  cfg_centroid_min_distance_ = config->get_float(CFG_PREFIX"centroid_min_distance");
  cfg_centroid_max_age_      = config->get_uint(CFG_PREFIX"centroid_max_age");
  cfg_syncpoint_in_          = config->get_string(CFG_PREFIX"syncpoint_in");
  cfg_syncpoint_out_         = config->get_string(CFG_PREFIX"syncpoint_out");

  try {
    pos_ifs_in_ = blackboard->open_multiple_for_reading<Position3DInterface>(cfg_ifs_in_.c_str());
    switch_if_ = blackboard->open_for_writing<SwitchInterface>("object-tracking");
    switch_if_->set_enabled(true);
    switch_if_->write();

    track_if_ = blackboard->open_for_writing<ObjectTrackingInterface>("object-tracking");
  } catch (Exception &e) {
    // close interface and rethrow
    for (list<Position3DInterface *>::iterator it = pos_ifs_in_.begin();
        it != pos_ifs_in_.end(); it++) {
      blackboard->close(*it);
    }
    blackboard->close(switch_if_);
    throw;
  }

  if (pos_ifs_in_.size() == 0) {
    throw Exception("Tried to open Interfaces %u, but none found.", cfg_ifs_in_.c_str());
  }

  try {
    double rotation[4] = {0., 0., 0., 1.};

    for (uint i = 1; i <= pos_ifs_in_.size(); i++) {
      char *tmp;
      if (asprintf(&tmp, "Tracked Object %u", i) != -1) {
        // Copy to get memory freed on exception
        std::string id = tmp;
        free(tmp);
        Position3DInterface *iface =
          blackboard->open_for_writing<Position3DInterface>(id.c_str());
        pos_ifs_out_[i] = iface;
        iface->set_rotation(rotation);
        iface->write();

        free_ids_.push_back(i);
      }
    }

  } catch (Exception &e) {
    // close interface and rethrow
    for (map<uint, Position3DInterface *>::iterator it = pos_ifs_out_.begin();
        it != pos_ifs_out_.end(); it++) {
      blackboard->close(it->second);
    }
    throw;
  }

  track_if_->set_num_names(free_ids_.size());
  track_if_->write();

  centroids_.clear();
  old_centroids_.clear();

}

void
ObjectTrackingThread::finalize()
{
  for(list<Position3DInterface *>::iterator it = pos_ifs_in_.begin(); it != pos_ifs_in_.end(); it++) {
    blackboard->close(*it);
  }
  for(map<uint, Position3DInterface *>::iterator it = pos_ifs_out_.begin(); it != pos_ifs_out_.end(); it++) {
    blackboard->close(it->second);
  }
  blackboard->close(switch_if_);

}

void
ObjectTrackingThread::loop()
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
      for (map<uint, Position3DInterface *>::iterator it = pos_ifs_out_.begin();
          it != pos_ifs_out_.end(); it++) {
        it->second->set_visibility_history(0);
        it->second->write();
      }
      switch_if_->set_enabled(false);
      switch_if_->write();
    }

    switch_if_->msgq_pop();
  }

  while (! track_if_->msgq_empty()) {
    if (ObjectTrackingInterface::SuggestIdMessage *msg =
        track_if_->msgq_first_safe(msg))
    {
      if (pos_ifs_out_.count(msg->obj_id())) {
        // interface exists already
        // delete the centroid from the currently visible centroids
        centroids_.erase(msg->obj_id());
        free_ids_.remove(msg->obj_id());
        prio_free_ids_.push_back(msg->obj_id());
      } else {
        char * tmp;
        if (asprintf(&tmp, "Tracked Object %u", msg->obj_id())) {
          std::string id = tmp;
          free(tmp);
          // interface does not exist, create it
          logger->log_warn(name(), "ID %u was suggested, but interface does not exist. "
              "Creating interface %s. "
              "Note that other components may not be able to use the interface"
              " (e.g. visualization won't show the object).", msg->obj_id(), id.c_str());
          try {
            Position3DInterface *iface = blackboard->open_for_writing<Position3DInterface>(id.c_str());
            iface->set_visibility_history(0);
            iface->write();
            pos_ifs_out_[msg->obj_id()] = iface;
            prio_free_ids_.push_back(msg->obj_id());
          } catch (Exception &e) {
            logger->log_error("Failed to create interface %s. Ignoring SuggestIdMessage for ID %u", id.c_str(), msg->obj_id());
          }
        } else {
          logger->log_error(name(), "Failed to create object name, ignoring SuggestIdMessage for ID %u", msg->id());
        }
      }
    }
    track_if_->msgq_pop();
  }

  if (! switch_if_->is_enabled()) {
    return;
  }

  // make sure we didn't 'lose' any IDs
  if (old_centroids_.size() + free_ids_.size() + centroids_.size() + prio_free_ids_.size() < pos_ifs_in_.size())
    logger->log_error(name(), "There are more input than output interfaces, cannot track all objects.");

  // read the frame id from the first centroid
  // this is later used when writing the new positions to the blackboard
  // we assume all centroids have the same frame id
  pos_ifs_in_.front()->read();
  frame_id_ = pos_ifs_in_.front()->frame();

  // read all positions and construct Vector4f centroids
  std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> new_centroids;
  for (list<Position3DInterface *>::iterator it = pos_ifs_in_.begin(); it != pos_ifs_in_.end(); it++) {
    (*it)->read();
    if ((*it)->visibility_history() < 0)
      continue;
    double * position = (*it)->translation();
    new_centroids.push_back(Eigen::Vector4f(position[0], position[1], position[2], 0.f));
  }


  // age all old centroids
  for (OldCentroidVector::iterator it = old_centroids_.begin();
      it != old_centroids_.end(); it++) {
    it->age();
  }

  std::map<unsigned int, int> assigned_ids = track_objects(new_centroids);

  centroids_.clear();

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

    centroids_[assigned_id] = new_centroids[i];

  }

  // delete centroids which are older than cfg_centroid_max_age_
  delete_old_centroids(old_centroids_, cfg_centroid_max_age_);
  // delete old centroids which are too close to current centroids
  delete_near_centroids(centroids_, old_centroids_, cfg_centroid_min_distance_);

  // set all pos_ifs not in centroids_ to 'not visible'
  for (map<uint, Position3DInterface *>::iterator it = pos_ifs_out_.begin(); it != pos_ifs_out_.end(); it++) {
    if (!centroids_.count(it->first)) {
      set_position(it->second, false);
    }
  }

  // set positions of all visible centroids
  for (CentroidMap::iterator it = centroids_.begin(); it != centroids_.end(); it++) {
    set_position(pos_ifs_out_[it->first], true, it->second, Eigen::Quaternionf(1, 0, 0, 0));
  }

  if (!centroids_.empty())
    first_run_ = false;

}

/**
 * calculate reassignment of IDs using the hungarian mehtod such that the total
 * distance all centroids moved is minimal
 * @param new_centroids current centroids which need correct ID assignment
 * @return map containing the new assignments, new_centroid -> ID
 * will be -1 if object is dropped, it's the caller's duty to remove any
 * reference to the centroid
 */
std::map<unsigned int, int>
ObjectTrackingThread::track_objects(
  const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> &new_centroids)
{
  std::map<uint, int> final_assignment;
  if (new_centroids.empty()) {
    for (CentroidMap::iterator it = centroids_.begin(); it != centroids_.end(); it++) {
      old_centroids_.push_back(OldCentroid(it->first, it->second));
    }
    return final_assignment;
  }
  if (first_run_) {
    // get a new id for every object since we didn't have objects before
    for (unsigned int i = 0; i < new_centroids.size(); i++) {
      final_assignment[i] = next_id();
    }
    return final_assignment;
  }
  else { // !first_run_
    hungarian_problem_t hp;
    // obj_ids: the id of the centroid in column i is saved in obj_ids[i]
    std::vector<unsigned int> obj_ids(centroids_.size());
    // create cost matrix,
    // save new centroids in rows, last centroids in columns
    // distance between new centroid i and last centroid j in cost[i][j]
    hp.num_rows = new_centroids.size();
    hp.num_cols = centroids_.size();
    hp.cost = (int**) calloc(hp.num_rows, sizeof(int*));
    for (int i = 0; i < hp.num_rows; i++)
      hp.cost[i] = (int*) calloc(hp.num_cols, sizeof(int));
    for (int row = 0; row < hp.num_rows; row++) { // new centroids
      unsigned int col = 0;
      for (CentroidMap::iterator col_it = centroids_.begin();
          col_it != centroids_.end(); col_it++, col++) { // old centroids
        double distance = pcl::distances::l2(new_centroids[row],
            col_it->second);
        hp.cost[row][col] = (int) (distance * 1000);
        obj_ids[col] = col_it->first;
      }
    }
    HungarianMethod solver;
    solver.init(hp.cost, hp.num_rows, hp.num_cols,
        HUNGARIAN_MODE_MINIMIZE_COST);
    solver.solve();
    // get assignments
    int assignment_size;
    int *assignment = solver.get_assignment(assignment_size);
    int id;
    for (int row = 0; row < assignment_size; row++) {
      if (row >= hp.num_rows) { // object has disappeared
        id = obj_ids.at(assignment[row]);
        old_centroids_.push_back(OldCentroid(id, centroids_.at(id)));
        continue;
      }
      else if (assignment[row] >= hp.num_cols) { // object is new or has reappeared
        bool assigned = false;
        // first, check if there is an old centroid close enough
        for (OldCentroidVector::iterator it = old_centroids_.begin();
            it != old_centroids_.end(); it++) {
          if (pcl::distances::l2(new_centroids[row], it->getCentroid())
              <= cfg_centroid_max_distance_) {
            id = it->getId();
            old_centroids_.erase(it);
            assigned = true;
            break;
          }
        }
        if (!assigned) {
          // we still don't have an id, create as new object
          id = next_id();
        }
      }
      else { // object has been assigned to an existing id
        id = obj_ids[assignment[row]];
        // check if centroid was moved further than cfg_centroid_max_distance_
        // this can happen if a centroid appears and another one disappears in the same loop
        // (then, the old centroid is assigned to the new one)
        if (pcl::distances::l2(centroids_[id], new_centroids[row])
            > cfg_centroid_max_distance_) {
          // save the (old) centroid because we don't use it now
          old_centroids_.push_back(OldCentroid(id, centroids_[id]));
          id = -1;
          // first check all old centroids if this is an old centroid reappearing
          for (OldCentroidVector::iterator it = old_centroids_.begin();
              it != old_centroids_.end(); it++) {
            if (pcl::distances::l2(new_centroids[row], it->getCentroid())
                <= cfg_centroid_max_distance_) {
              id = it->getId();
              old_centroids_.erase(it);
              break;
            }
          }
          // assign a new id if no old centroid was found
          if (id == -1) {
            id = next_id();
          }
        }
      }
      final_assignment[row] = id;
    }
    return final_assignment;
  }
}

int ObjectTrackingThread::next_id() {
  if (!prio_free_ids_.empty()) {
    int id = prio_free_ids_.front();
    prio_free_ids_.pop_front();
    logger->log_debug(name(), "Assigning new object to suggested ID %u", id);
    return id;
  }
  if (free_ids_.empty()) {
    logger->log_debug(name(), "free_ids is empty");
    return -1;
  }
  int id = free_ids_.front();
  free_ids_.pop_front();
  return id;
}

void
ObjectTrackingThread::set_position(fawkes::Position3DInterface *iface,
                                    bool is_visible,
                                    const Eigen::Vector4f &centroid,
                                    const Eigen::Quaternionf &attitude)
{
  tf::Stamped<tf::Pose> spose = centroid_to_pose(centroid, attitude, frame_id_);
  iface->set_frame(frame_id_.c_str());

  set_pos_interface(iface, is_visible, spose);
}

void ObjectTrackingThread::delete_old_centroids(OldCentroidVector &centroids,
  unsigned int age)
{
  centroids.erase(
      std::remove_if(
        centroids.begin(),
        centroids.end(),
        [&](const OldCentroid &centroid)->bool {
    if (centroid.getAge() > age) {
      free_ids_.push_back(centroid.getId());
      return true;
    }
    return false;
  }), centroids.end());
}

void ObjectTrackingThread::delete_near_centroids(const CentroidMap &reference,
  OldCentroidVector &centroids, float min_distance)
{
  centroids.erase(
      std::remove_if(
        centroids.begin(),
        centroids.end(),
        [&](const OldCentroid &old)->bool {
    for (CentroidMap::const_iterator it = reference.begin(); it != reference.end(); it++) {
      if (pcl::distances::l2(it->second, old.getCentroid()) < min_distance) {
        free_ids_.push_back(old.getId());
        return true;
      }
    }
    return false;
  }), centroids.end());
}
