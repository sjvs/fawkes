
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
  BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_SENSOR_PROCESS),
  SyncPointAspect(SyncPoint::WAIT_FOR_ALL, "/perception", "")
{
}


void
TabletopVisualizationThread::init()
{

  // allow concurrent execution of prepare_finalize and loop
  // this is necessary to interrupt syncpoint wait calls
  set_prepfin_conc_loop(true);

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
  cfg_show_cvxhull_line_highlighting_ = true;
  try {
    cfg_show_cvxhull_line_highlighting_ = config->get_bool(CFG_PREFIX_VIS"show_convex_hull_line_highlighting");
  } catch (Exception &e) {} // ignored, use default
  cfg_show_cvxhull_vertices_ = true;
  try {
    cfg_show_cvxhull_vertices_ = config->get_bool(CFG_PREFIX_VIS"show_convex_hull_vertices");
  } catch (Exception &e) {} // ignored, use default
  cfg_show_cvxhull_vertex_ids_ = true;
  try {
    cfg_show_cvxhull_vertex_ids_ = config->get_bool(CFG_PREFIX_VIS"show_convex_hull_vertex_ids");
  } catch (Exception &e) {} // ignored, use default
  cfg_show_frustrum_ = false;
  try {
    cfg_show_frustrum_ = config->get_bool(CFG_PREFIX_VIS"show_frustrum");
  } catch (Exception &e) {} // ignored, use default
  if (cfg_show_frustrum_) {
    cfg_horizontal_va_ = deg2rad(config->get_float(CFG_PREFIX_VIS"horizontal_viewing_angle"));
    cfg_vertical_va_   = deg2rad(config->get_float(CFG_PREFIX_VIS"vertical_viewing_angle"));
  }


  vispub_ = new ros::Publisher();
  *vispub_ = rosnode->advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 100);
//#ifdef USE_POSEPUB
//  posepub_ = new ros::Publisher();
//  *posepub_ = rosnode->advertise<geometry_msgs::PointStamped>("table_point", 10);
//#endif
  last_id_num_ = 0;

  obj_pos_ifs_ = blackboard->open_multiple_for_reading<Position3DInterface>(cfg_object_name_pattern_.c_str());
  if (obj_pos_ifs_.size() == 0) {
    throw Exception("Tried to open interfaces %u, but none found.", cfg_object_name_pattern_.c_str());
  }
  table_pos_if_ = blackboard->open_for_reading<Position3DInterface>("Tabletop");
  hull_if_ = blackboard->open_for_reading<TabletopHullInterface>("tabletop-hull");
  model_hull_if_ = blackboard->open_for_reading<TabletopHullInterface>("tabletop-model-hull");
  good_hull_edges_if_ = blackboard->open_for_reading<TabletopEdgesInterface>("tabletop-good-edges");

  switch_if_ = blackboard->open_for_writing<SwitchInterface>("tabletop-visualization");
  switch_if_->set_enabled(true);
  switch_if_->write();

}

void
TabletopVisualizationThread::finalize()
{
  delete_all_markers();
  vispub_->shutdown();
  delete vispub_;
//#ifdef USE_POSEPUB
//  posepub_->shutdown();
//  delete posepub_;
//#endif

  for (list<Position3DInterface *>::iterator it = obj_pos_ifs_.begin(); it != obj_pos_ifs_.end(); it++) {
    blackboard->close(*it);
  }
  blackboard->close(table_pos_if_);
  blackboard->close(switch_if_);
  blackboard->close(hull_if_);
  blackboard->close(model_hull_if_);
  blackboard->close(good_hull_edges_if_);

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
    delete_all_markers();
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
  if (table_normal[2] < 0.f) {
    table_normal *= -1;
  }
//  logger->log_debug(name(), "table_normal: (%f %f %f)", table_normal3[0], table_normal3[1], table_normal3[2]);
  Eigen::Vector4f normal_end = (table_centroid + (table_normal * 0.15));
//  logger->log_debug(name(), "normal_end: (%f %f %f)", normal_end[0], normal_end[1], normal_end[2]);

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


  bool have_hull_markers = false;
  if (cfg_show_cvxhull_line_highlighting_) {
    good_hull_edges_if_->read();
    // "Good" lines are highlighted
    visualization_msgs::Marker hull_lines;
    hull_lines.header.frame_id = good_hull_edges_if_->frame();
    hull_lines.header.stamp = ros::Time::now();
    hull_lines.ns = "tabletop";
    hull_lines.id = idnum++;
    hull_lines.type = visualization_msgs::Marker::LINE_LIST;
    hull_lines.action = visualization_msgs::Marker::ADD;
    hull_lines.points.resize(good_hull_edges_if_->num_points());
    hull_lines.colors.resize(good_hull_edges_if_->num_points());
    for (size_t i = 0; i < good_hull_edges_if_->num_points(); ++i) {
      hull_lines.points[i].x = good_hull_edges_if_->x(i);
      hull_lines.points[i].y = good_hull_edges_if_->y(i);
      hull_lines.points[i].z = good_hull_edges_if_->z(i);
      hull_lines.colors[i].r = 0.;
      hull_lines.colors[i].b = 0.;
      hull_lines.colors[i].a = 0.4;
      if (good_hull_edges_if_->goodness(i) > 0.) {
        hull_lines.colors[i].g = 1.0;
      } else {
        hull_lines.colors[i].g = 0.5;
      }
    }
    hull_lines.color.a = 1.0;
    hull_lines.scale.x = 0.01;
    hull_lines.lifetime = ros::Duration(cfg_duration_, 0);
    m.markers.push_back(hull_lines);
    have_hull_markers = true;
  }

  // Table model surrounding polygon
  model_hull_if_->read();
  if (model_hull_if_->num_points() > 0) {
    //    logger->log_debug(name(), "visualizing model hull");
    m.markers.push_back(visualize_hull(model_hull_if_, idnum));
    have_hull_markers = true;
  }

  hull_if_->read();
  if (!have_hull_markers && hull_if_->num_points() > 0) {
    //    logger->log_debug(name(), "visualizing hull");
    m.markers.push_back(visualize_hull(hull_if_, idnum));
  }

  if (cfg_show_cvxhull_vertices_) {
    m.markers.push_back(visualize_cvxhull_vertices(hull_if_, idnum));
  }

  // hull texts
  if (cfg_show_cvxhull_vertex_ids_) {
    visualization_msgs::MarkerArray text_markers;
    for (size_t i = 0; i < hull_if_->num_points(); ++i) {

      char *tmp;
      if (asprintf(&tmp, "Cvx_%zu", i) != -1) {
        // Copy to get memory freed on exception
        std::string id = tmp;
        free(tmp);

        visualization_msgs::Marker text;
        text.header.frame_id = hull_if_->frame();
        text.header.stamp = ros::Time::now();
        text.ns = "tabletop";
        text.id = idnum++;
        text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text.action = visualization_msgs::Marker::ADD;
        text.pose.position.x = hull_if_->x(i);
        text.pose.position.y = hull_if_->y(i);
        text.pose.position.z = hull_if_->z(i) + 0.1;
        text.pose.orientation.w = 1.;
        text.scale.z = 0.03;
        text.color.r = text.color.g = text.color.b = 1.0f;
        text.color.a = 1.0;
        text.lifetime = ros::Duration(cfg_duration_, 0);
        text.text = id;
        m.markers.push_back(text);
      }
    }
  }

  if (model_hull_if_->num_points() == 4) {
    m.markers.push_back(visualize_plane(model_hull_if_, idnum));
  }

  if (cfg_show_frustrum_ && model_hull_if_->num_points() > 0) {
    visualization_msgs::Marker frustrum;
    frustrum.header.frame_id = model_hull_if_->frame();
    frustrum.header.stamp = ros::Time::now();
    frustrum.ns = "tabletop";
    frustrum.id = idnum++;
    frustrum.type = visualization_msgs::Marker::LINE_LIST;
    frustrum.action = visualization_msgs::Marker::ADD;
    frustrum.points.resize(8);
    frustrum.points[0].x = frustrum.points[2].x = frustrum.points[4].x = frustrum.points[6].x = 0.;
    frustrum.points[0].y = frustrum.points[2].y = frustrum.points[4].y = frustrum.points[6].y = 0.;
    frustrum.points[0].z = frustrum.points[2].z = frustrum.points[4].z = frustrum.points[6].z = 0.;

    const float half_hva = cfg_horizontal_va_ * 0.5;
    const float half_vva = cfg_vertical_va_ * 0.5;

    Eigen::Matrix3f upper_right_m;
    upper_right_m =
        Eigen::AngleAxisf(-half_hva, Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(-half_vva, Eigen::Vector3f::UnitY());
    Eigen::Vector3f upper_right = upper_right_m * Eigen::Vector3f(4,0,0);

    Eigen::Matrix3f upper_left_m;
    upper_left_m =
        Eigen::AngleAxisf(half_hva, Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(-half_vva, Eigen::Vector3f::UnitY());
    Eigen::Vector3f upper_left = upper_left_m * Eigen::Vector3f(4,0,0);

    Eigen::Matrix3f lower_right_m;
    lower_right_m =
        Eigen::AngleAxisf(-half_hva, Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(half_vva, Eigen::Vector3f::UnitY());
    Eigen::Vector3f lower_right = lower_right_m * Eigen::Vector3f(2,0,0);

    Eigen::Matrix3f lower_left_m;
    lower_left_m =
        Eigen::AngleAxisf(half_hva, Eigen::Vector3f::UnitZ())
    * Eigen::AngleAxisf(half_vva, Eigen::Vector3f::UnitY());
    Eigen::Vector3f lower_left = lower_left_m * Eigen::Vector3f(2,0,0);

    frustrum.points[1].x = upper_right[0];
    frustrum.points[1].y = upper_right[1];
    frustrum.points[1].z = upper_right[2];

    frustrum.points[3].x = lower_right[0];
    frustrum.points[3].y = lower_right[1];
    frustrum.points[3].z = lower_right[2];

    frustrum.points[5].x = lower_left[0];
    frustrum.points[5].y = lower_left[1];
    frustrum.points[5].z = lower_left[2];

    frustrum.points[7].x = upper_left[0];
    frustrum.points[7].y = upper_left[1];
    frustrum.points[7].z = upper_left[2];

    frustrum.scale.x = 0.005;
    frustrum.color.r = 1.0;
    frustrum.color.g = frustrum.color.b = 0.f;
    frustrum.color.a = 1.0;
    frustrum.lifetime = ros::Duration(cfg_duration_, 0);

    m.markers.push_back(frustrum);



    visualization_msgs::Marker frustrum_triangles;
    frustrum_triangles.header.frame_id = model_hull_if_->frame();
    frustrum_triangles.header.stamp = ros::Time::now();
    frustrum_triangles.ns = "tabletop";
    frustrum_triangles.id = idnum++;
    frustrum_triangles.type = visualization_msgs::Marker::TRIANGLE_LIST;
    frustrum_triangles.action = visualization_msgs::Marker::ADD;
    frustrum_triangles.points.resize(9);
    frustrum_triangles.points[0].x =
        frustrum_triangles.points[3].x = frustrum_triangles.points[6].x = 0.;
    frustrum_triangles.points[0].y =
        frustrum_triangles.points[3].y = frustrum_triangles.points[3].y = 0.;
    frustrum_triangles.points[0].z
    = frustrum_triangles.points[3].z = frustrum_triangles.points[3].z = 0.;

    frustrum_triangles.points[1].x = upper_right[0];
    frustrum_triangles.points[1].y = upper_right[1];
    frustrum_triangles.points[1].z = upper_right[2];

    frustrum_triangles.points[2].x = lower_right[0];
    frustrum_triangles.points[2].y = lower_right[1];
    frustrum_triangles.points[2].z = lower_right[2];

    frustrum_triangles.points[4].x = lower_left[0];
    frustrum_triangles.points[4].y = lower_left[1];
    frustrum_triangles.points[4].z = lower_left[2];

    frustrum_triangles.points[5].x = upper_left[0];
    frustrum_triangles.points[5].y = upper_left[1];
    frustrum_triangles.points[5].z = upper_left[2];

    frustrum_triangles.points[7].x = lower_left[0];
    frustrum_triangles.points[7].y = lower_left[1];
    frustrum_triangles.points[7].z = lower_left[2];

    frustrum_triangles.points[8].x = lower_right[0];
    frustrum_triangles.points[8].y = lower_right[1];
    frustrum_triangles.points[8].z = lower_right[2];

    frustrum_triangles.scale.x = 1;
    frustrum_triangles.scale.y = 1;
    frustrum_triangles.scale.z = 1;
    frustrum_triangles.color.r = 1.0;
    frustrum_triangles.color.g = frustrum_triangles.color.b = 0.f;
    frustrum_triangles.color.a = 0.23;
    frustrum_triangles.lifetime = ros::Duration(cfg_duration_, 0);

    m.markers.push_back(frustrum_triangles);
  }


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

//#ifdef USE_POSEPUB
//  geometry_msgs::PointStamped p;
//  p.header.frame_id = frame_id_;
//  p.header.stamp = ros::Time::now();
//  p.point.x = table_centroid_[0];
//  p.point.y = table_centroid_[1];
//  p.point.z = table_centroid_[2];
//  posepub_->publish(p);
//#endif
}

visualization_msgs::Marker
TabletopVisualizationThread::visualize_hull(TabletopHullInterface *hull_if, unsigned int &idnum)
{
  visualization_msgs::Marker hull;
  hull.header.frame_id = hull_if->frame();
  hull.header.stamp = ros::Time::now();
  hull.ns = "tabletop";
  hull.id = idnum++;
  hull.type = visualization_msgs::Marker::LINE_STRIP;
  hull.action = visualization_msgs::Marker::ADD;
  hull.points.resize(hull_if->num_points() + 1);
  for (size_t i = 0; i < hull_if->num_points(); ++i) {
    hull.points[i].x = hull_if->x(i);
    hull.points[i].y = hull_if->y(i);
    hull.points[i].z = hull_if->z(i);
  }
  hull.points[hull_if->num_points()].x = hull_if->x(0);
  hull.points[hull_if->num_points()].y = hull_if->y(0);
  hull.points[hull_if->num_points()].z = hull_if->z(0);
  hull.scale.x = 0.005;
  hull.color.r = 0.4;
  hull.color.g = hull.color.b = 0.f;
  hull.color.a = 0.2;
  hull.lifetime = ros::Duration(cfg_duration_, 0);
  return(hull);
}

visualization_msgs::Marker
TabletopVisualizationThread::visualize_cvxhull_vertices(TabletopHullInterface* hull_if, uint &idnum)
{
  visualization_msgs::Marker hull_points;
  hull_points.header.frame_id = hull_if->frame();
  hull_points.header.stamp = ros::Time::now();
  hull_points.ns = "tabletop";
  hull_points.id = idnum++;
  hull_points.type = visualization_msgs::Marker::SPHERE_LIST;
  hull_points.action = visualization_msgs::Marker::ADD;
  hull_points.points.resize(hull_if->num_points());
  for (size_t i = 0; i < hull_if->num_points(); ++i) {
    hull_points.points[i].x = hull_if->x(i);
    hull_points.points[i].y = hull_if->y(i);
    hull_points.points[i].z = hull_if->z(i);
  }
  hull_points.scale.x = 0.01;
  hull_points.scale.y = 0.01;
  hull_points.scale.z = 0.01;
  hull_points.color.r = 0.8;
  hull_points.color.g = hull_points.color.b = 0.f;
  hull_points.color.a = 1.0;
  hull_points.lifetime = ros::Duration(cfg_duration_, 0);
  return hull_points;
}


visualization_msgs::Marker
TabletopVisualizationThread::visualize_plane(
  fawkes::TabletopHullInterface* model_hull_if, uint &idnum)
{
  visualization_msgs::Marker plane;
  plane.header.frame_id = model_hull_if->frame();
  plane.header.stamp = ros::Time::now();
  plane.ns = "tabletop";
  plane.id = idnum++;
  plane.type = visualization_msgs::Marker::TRIANGLE_LIST;
  plane.action = visualization_msgs::Marker::ADD;
  plane.points.resize(6);
  for (unsigned int i = 0; i < 3; ++i) {
    plane.points[i].x = model_hull_if->x(i);
    plane.points[i].y = model_hull_if->y(i);
    plane.points[i].z = model_hull_if->z(i);
  }
  for (unsigned int i = 2; i < 5; ++i) {
    plane.points[i + 1].x = model_hull_if->x(i % 4);
    plane.points[i + 1].y = model_hull_if->y(i % 4);
    plane.points[i + 1].z = model_hull_if->z(i % 4);
  }
  plane.pose.orientation.w = 1.;
  plane.scale.x = 1.0;
  plane.scale.y = 1.0;
  plane.scale.z = 1.0;
  plane.color.r = ((float) table_color[0] / 255.f) * 0.8;
  plane.color.g = ((float) table_color[1] / 255.f) * 0.8;
  plane.color.b = ((float) table_color[2] / 255.f) * 0.8;
  plane.color.a = 1.0;
  plane.lifetime = ros::Duration(cfg_duration_, 0);
  return plane;
}

void
TabletopVisualizationThread::delete_all_markers()
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
  last_id_num_ = 0;
}
