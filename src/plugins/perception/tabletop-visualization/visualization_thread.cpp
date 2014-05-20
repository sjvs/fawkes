
/***************************************************************************
 *  visualization_thread.cpp - Visualization via rviz
 *
 *  Created: Fri Nov 11 00:20:45 2011
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

#include "visualization_thread.h"
#include "../common/cluster_colors.h"
#include "../common/perception_common.h"

#include <core/threading/mutex_locker.h>
#include <utils/math/angle.h>
#include <utils/time/wait.h>

#include <libs/syncpoint/exceptions.h>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#ifdef USE_POSEPUB
#  include <geometry_msgs/PointStamped.h>
#endif
#include <Eigen/Geometry>

extern "C" {
#ifdef HAVE_QHULL_2011
#  include "libqhull/libqhull.h"
#  include "libqhull/mem.h"
#  include "libqhull/qset.h"
#  include "libqhull/geom.h"
#  include "libqhull/merge.h"
#  include "libqhull/poly.h"
#  include "libqhull/io.h"
#  include "libqhull/stat.h"
#else
#  include "qhull/qhull.h"
#  include "qhull/mem.h"
#  include "qhull/qset.h"
#  include "qhull/geom.h"
#  include "qhull/merge.h"
#  include "qhull/poly.h"
#  include "qhull/io.h"
#  include "qhull/stat.h"
#endif
}

//#define CFG_PREFIX "/perception/tabletop-objects/"
#define CFG_PREFIX_TABLE_REC "/perception/tabletop-recognition"
#define CFG_PREFIX_VIS "/perception/visualization/"

using namespace std;
using namespace fawkes;
using namespace fawkes::perception;

/** @class TabletopVisualizationThread "visualization_thread.h"
 * Send Marker messages to rviz.
 * This class takes input from the table top object detection thread and
 * publishes according marker messages for visualization in rviz.
 * @author Tim Niemueller
 */

/** Constructor. */
TabletopVisualizationThread::TabletopVisualizationThread()
: fawkes::Thread("TabletopVisualizationThread", Thread::OPMODE_WAITFORWAKEUP),
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS)
{
}


void
TabletopVisualizationThread::init()
{
  cfg_duration_ = 120;
  try {
    cfg_duration_ = config->get_uint(CFG_PREFIX_VIS"display_duration");
  } catch (Exception &e) {} // ignored, use default
  cfg_object_name_pattern_ = "Tabletop Object *";
  try {
    cfg_object_name_pattern_ = config->get_string(CFG_PREFIX_VIS"object_name_pattern");
  } catch (Exception &e) {} // ignored, use default
  cfg_syncpoint_ = "/perception/object-detection";
  try {
    cfg_syncpoint_ = config->get_string(CFG_PREFIX_VIS"syncpoint");
  } catch (Exception &e) {} // ignored, use default

  vispub_ = new ros::Publisher();
  *vispub_ = rosnode->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
#ifdef USE_POSEPUB
  posepub_ = new ros::Publisher();
  *posepub_ = rosnode->advertise<geometry_msgs::PointStamped>("table_point", 10);
#endif
  last_id_num_ = 0;

  obj_pos_ifs_ = blackboard->open_multiple_for_reading<Position3DInterface>(cfg_object_name_pattern_.c_str());
  table_pos_if_ = blackboard->open_for_reading<Position3DInterface>("Tabletop");

  syncpoint_ = syncpoint_manager->get_syncpoint(name(), cfg_syncpoint_.c_str());

  switch_if_ = blackboard->open_for_writing<SwitchInterface>("tabletop-visualization");
  switch_if_->set_enabled(true);
  switch_if_->write();

}

void
TabletopVisualizationThread::finalize()
{
  visualization_msgs::MarkerArray m;
  // delete old markers
  for (size_t i = 0; i < last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = frame_id_;
    delop.header.stamp = ros::Time::now();
    delop.ns = "tabletop";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }
  vispub_->publish(m);

  vispub_->shutdown();
  delete vispub_;
#ifdef USE_POSEPUB
  posepub_->shutdown();
  delete posepub_;
#endif

  for (list<Position3DInterface *>::iterator it = obj_pos_ifs_.begin(); it != obj_pos_ifs_.end(); it++) {
    blackboard->close(*it);
  }
  blackboard->close(table_pos_if_);
  blackboard->close(switch_if_);

  syncpoint_manager->release_syncpoint(name(), syncpoint_);
}


void
TabletopVisualizationThread::loop()
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

  try {
    syncpoint_->wait(name());
  } catch (const SyncPointMultipleWaitCallsException &e) {
    logger->log_warn(name(), "Tried to run, but already running.");
    return;
  }

  MutexLocker lock(&mutex_);
  visualization_msgs::MarkerArray m;

  table_pos_if_->read();
  std::string table_frame_id = table_pos_if_->frame();

  // save frame id for deletion
  frame_id_ = table_frame_id;

  unsigned int idnum = 0;
  unsigned int id = 0;
  for (list<Position3DInterface *>::iterator it = obj_pos_ifs_.begin(); it != obj_pos_ifs_.end(); ++id, it++) {
    std::string frame_id;
    try {
      (*it)->read();
      if ((*it)->visibility_history() <= 0) continue;
      frame_id = (*it)->frame();
      tf::Stamped<tf::Point>
        centroid(tf::Point((*it)->translation(0), (*it)->translation(1), (*it)->translation(2)),
                 fawkes::Time(0, 0), frame_id);
      tf::Stamped<tf::Point> camrel_centroid;
      tf_listener->transform_point(table_frame_id, centroid, camrel_centroid);

      char *tmp;
      if (asprintf(&tmp, "TObj %u", id) != -1) {
        // Copy to get memory freed on exception
        std::string id = tmp;
        free(tmp);

        visualization_msgs::Marker text;
        text.header.frame_id = table_frame_id;
        text.header.stamp = ros::Time::now();
        text.ns = "tabletop";
        text.id = idnum++;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        /*        text.pose.position.x = baserel_centroid[0];
         text.pose.position.y = baserel_centroid[1];
         text.pose.position.z = baserel_centroid[2] + 0.17;*/
        text.pose.position.x = centroid[0];
        text.pose.position.y = centroid[1];
        text.pose.position.z = centroid[2] + 0.17;
        text.pose.orientation.w = 1.;
        text.scale.z = 0.05; // 5cm high
        text.color.r = text.color.g = text.color.b = 1.0f;
        text.color.a = 1.0;
        text.lifetime = ros::Duration(cfg_duration_, 0);
        text.text = id;
        m.markers.push_back(text);
      }

      visualization_msgs::Marker sphere;
      sphere.header.frame_id = table_frame_id;
      sphere.header.stamp = ros::Time::now();
      sphere.ns = "tabletop";
      sphere.id = idnum++;
      sphere.type = visualization_msgs::Marker::CYLINDER;
      sphere.action = visualization_msgs::Marker::ADD;

      sphere.pose.position.x = centroid[0];
      sphere.pose.position.y = centroid[1];
      sphere.pose.position.z = centroid[2];
      sphere.pose.orientation.w = 1.;
      sphere.scale.x = sphere.scale.y = 0.08;
      sphere.scale.z = 0.09;
      sphere.color.r = (float)cluster_colors[id % MAX_CENTROIDS][0] / 255.f;
      sphere.color.g = (float)cluster_colors[id % MAX_CENTROIDS][1] / 255.f;
      sphere.color.b = (float)cluster_colors[id % MAX_CENTROIDS][2] / 255.f;
      sphere.color.a = 1.0;
      sphere.lifetime = ros::Duration(cfg_duration_, 0);
      m.markers.push_back(sphere);
    } catch (Exception &e) {
      logger->log_debug(name(), "Exception during visualization: %s", e.what());
    }
  }

  Eigen::Vector4f table_centroid = Eigen::Vector4f(table_pos_if_->translation(0),
    table_pos_if_->translation(1), table_pos_if_->translation(2), 0.f);
  Eigen::Quaternionf quat = { static_cast<float>(table_pos_if_->rotation(0)), static_cast<float>(table_pos_if_->rotation(1)),
      static_cast<float>(table_pos_if_->rotation(2)), static_cast<float>(table_pos_if_->rotation(3)) };
  Eigen::Vector3f table_normal3 = quaternion_to_normal(quat);

  Eigen::Vector4f table_normal(table_normal3[0], table_normal3[1], table_normal3[2], 0.f);

  Eigen::Vector4f normal_end = (table_centroid + (table_normal * -0.15));

  visualization_msgs::Marker normal;
  normal.header.frame_id = table_frame_id;
  normal.header.stamp = ros::Time::now();
  normal.ns = "tabletop";
  normal.id = idnum++;
  normal.type = visualization_msgs::Marker::ARROW;
  normal.action = visualization_msgs::Marker::ADD;
  normal.points.resize(2);
  normal.points[0].x = table_centroid[0];
  normal.points[0].y = table_centroid[1];
  normal.points[0].z = table_centroid[2];
  normal.points[1].x = normal_end[0];
  normal.points[1].y = normal_end[1];
  normal.points[1].z = normal_end[2];
  normal.scale.x = 0.02;
  normal.scale.y = 0.04;
  normal.color.r = 0.4;
  normal.color.g = normal.color.b = 0.f;
  normal.color.a = 1.0;
  normal.lifetime = ros::Duration(cfg_duration_, 0);
  m.markers.push_back(normal);


  // delete old markers
  for (size_t i = idnum; i < last_id_num_; ++i) {
    visualization_msgs::Marker delop;
    delop.header.frame_id = frame_id_;
    delop.header.stamp = ros::Time::now();
    delop.ns = "tabletop";
    delop.id = i;
    delop.action = visualization_msgs::Marker::DELETE;
    m.markers.push_back(delop);
  }

  last_id_num_ = idnum;

  vispub_->publish(m);

#ifdef USE_POSEPUB
  geometry_msgs::PointStamped p;
  p.header.frame_id = frame_id_;
  p.header.stamp = ros::Time::now();
  p.point.x = table_centroid_[0];
  p.point.y = table_centroid_[1];
  p.point.z = table_centroid_[2];
  posepub_->publish(p);
#endif
}

