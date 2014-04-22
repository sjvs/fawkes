/***************************************************************************
 *  perception_common.h - common functions shared by perception plugins
 *
 *  Created: Thu Apr 17 12:34:22 2014
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

#ifndef __PLUGINS_PERCEPTION_COMMON_H_
#define __PLUGINS_PERCEPTION_COMMON_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/types.h>
#include <interfaces/Position3DInterface.h>

#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <string>

namespace fawkes {
namespace perception {
#if 0 /* just to make Emacs auto-indent happy */
}
}
#endif

typedef pcl::PointXYZ PointType;
typedef pcl::PointCloud<PointType> Cloud;

typedef pcl::PointXYZRGB ColorPointType;
typedef pcl::PointCloud<ColorPointType> ColorCloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

typedef ColorCloud::Ptr ColorCloudPtr;
typedef ColorCloud::ConstPtr ColorCloudConstPtr;

void set_pos_interface(fawkes::Position3DInterface *iface, bool is_visible,
  tf::Stamped<tf::Pose> baserel_pose =
      tf::Stamped<tf::Pose>(tf::Pose(), fawkes::Time(0, 0), ""));
ColorCloudPtr colorize_cluster(CloudConstPtr input_cloud,
  const std::vector<int> &cluster, const uint8_t color[]);
tf::Stamped<tf::Pose> centroid_to_pose(const Eigen::Vector4f &centroid,
  const Eigen::Quaternionf &attitude, const std::string &frame);
Eigen::Quaternionf normal_to_quaternion(Eigen::Vector3f normal,
  Eigen::Vector3f reference = Eigen::Vector3f(0.f, 0.f, 1.f));
Eigen::Vector3f quaternion_to_normal(Eigen::Quaternionf quat,
  Eigen::Vector3f reference = Eigen::Vector3f(0.f, 0.f, 1.f));

}
}

#endif
