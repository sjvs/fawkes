/***************************************************************************
 *  perception_common.cpp - common functions shared by perception plugins
 *
 *  Created: Thu Apr 17 12:38:33 2014
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

#include "perception_common.h"

using namespace std;

namespace fawkes {
namespace perception {
#if 0 /* just to make Emacs auto-indent happy */
}
}
#endif

tf::Stamped<tf::Pose>
centroid_to_pose(const Eigen::Vector4f &centroid,
  const Eigen::Quaternionf &attitude, const string &frame)
{
  return tf::Stamped<tf::Pose>(
      tf::Pose(
          tf::Quaternion(attitude.x(), attitude.y(), attitude.z(),
              attitude.w()),
          tf::Vector3(centroid[0], centroid[1], centroid[2])),
      fawkes::Time(0, 0), frame);
}



void
set_pos_interface(fawkes::Position3DInterface *iface, bool is_visible,
  tf::Stamped<tf::Pose> baserel_pose)
{
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
  } else { // !is_visible
    if (visibility_history <= 0) {
      iface->set_visibility_history(visibility_history - 1);
    } else {
      iface->set_visibility_history(-1);
    }
  }

  iface->write();
}


ColorCloudPtr colorize_cluster (
    CloudConstPtr input_cloud,
    const std::vector<int> &cluster,
    const uint8_t color[]) {
  ColorCloudPtr result(new ColorCloud());
  result->resize(cluster.size());
  result->header.frame_id = input_cloud->header.frame_id;
  uint i = 0;
  for (std::vector<int>::const_iterator it = cluster.begin(); it != cluster.end(); ++it, ++i) {
    ColorPointType &p1 = result->points.at(i);
    const PointType &p2 = input_cloud->points.at(*it);
    p1.x = p2.x;
    p1.y = p2.y;
    p1.z = p2.z;
    p1.r = color[0];
    p1.g = color[1];
    p1.b = color[2];
  }
  return result;
}

Eigen::Quaternionf normal_to_quaternion(Eigen::Vector3f normal,
  Eigen::Vector3f reference ) {
  Eigen::Quaternionf quat;
  quat.setFromTwoVectors(reference, normal);
  return quat;
}

Eigen::Vector3f quaternion_to_normal(Eigen::Quaternionf quat,
  Eigen::Vector3f reference) {
  return quat._transformVector(reference);
}

} // namespace perception
} // namespace fawkes
