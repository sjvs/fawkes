
/***************************************************************************
 *  object.cpp - Fawkes BlackBoard Interface - ObjectPositionInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2007-2008  Tim Niemueller
 *
 *  $Id$
 *
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version. A runtime exception applies to
 *  this software (see LICENSE.GPL_WRE file mentioned below for details).
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL_WRE file in the doc directory.
 */

#include <interfaces/object.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class ObjectPositionInterface <interfaces/object.h>
 * ObjectPositionInterface Fawkes BlackBoard Interface.
 * 
      This interface provides access to arbitrary object positions and velocities. You
      can use it to store the position of any object in the RoboCup domain. There is a type
      indicator for the RoboCup soccer domain to easily distinguish several well
      known objects. You may choose not to use this for other application in which case
      the value should be other (which is also the default).
    
 */


/** TYPE_OTHER constant */
const unsigned int ObjectPositionInterface::TYPE_OTHER = 0;
/** TYPE_BALL constant */
const unsigned int ObjectPositionInterface::TYPE_BALL = 1;
/** TYPE_OPPONENT constant */
const unsigned int ObjectPositionInterface::TYPE_OPPONENT = 2;
/** TYPE_TEAMMEMBER constant */
const unsigned int ObjectPositionInterface::TYPE_TEAMMEMBER = 3;
/** TYPE_LINE constant */
const unsigned int ObjectPositionInterface::TYPE_LINE = 4;
/** TYPE_SELF constant */
const unsigned int ObjectPositionInterface::TYPE_SELF = 5;
/** TYPE_GOAL_BLUE constant */
const unsigned int ObjectPositionInterface::TYPE_GOAL_BLUE = 6;
/** TYPE_GOAL_YELLOW constant */
const unsigned int ObjectPositionInterface::TYPE_GOAL_YELLOW = 7;
/** FLAG_NONE constant */
const unsigned int ObjectPositionInterface::FLAG_NONE = 0;
/** FLAG_HAS_WORLD constant */
const unsigned int ObjectPositionInterface::FLAG_HAS_WORLD = 1;
/** FLAG_HAS_RELATIVE_CARTESIAN constant */
const unsigned int ObjectPositionInterface::FLAG_HAS_RELATIVE_CARTESIAN = 2;
/** FLAG_HAS_RELATIVE_POLAR constant */
const unsigned int ObjectPositionInterface::FLAG_HAS_RELATIVE_POLAR = 4;
/** FLAG_HAS_EULER_ANGLES constant */
const unsigned int ObjectPositionInterface::FLAG_HAS_EULER_ANGLES = 8;
/** FLAG_HAS_EXTENT constant */
const unsigned int ObjectPositionInterface::FLAG_HAS_EXTENT = 16;
/** FLAG_HAS_VOLUME_EXTENT constant */
const unsigned int ObjectPositionInterface::FLAG_HAS_VOLUME_EXTENT = 32;
/** FLAG_HAS_CIRCULAR_EXTENT constant */
const unsigned int ObjectPositionInterface::FLAG_HAS_CIRCULAR_EXTENT = 64;
/** FLAG_HAS_COVARIANCES constant */
const unsigned int ObjectPositionInterface::FLAG_HAS_COVARIANCES = 128;
/** FLAG_HAS_WORLD_VELOCITY constant */
const unsigned int ObjectPositionInterface::FLAG_HAS_WORLD_VELOCITY = 256;
/** FLAG_HAS_Z_AS_ORI constant */
const unsigned int ObjectPositionInterface::FLAG_HAS_Z_AS_ORI = 512;
/** FLAG_IS_FIXED_OBJECT constant */
const unsigned int ObjectPositionInterface::FLAG_IS_FIXED_OBJECT = 1024;

/** Constructor */
ObjectPositionInterface::ObjectPositionInterface() : Interface()
{
  data_size = sizeof(ObjectPositionInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (ObjectPositionInterface_data_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(Interface::IFT_UINT, "object_type", 1, &data->object_type);
  add_fieldinfo(Interface::IFT_UINT, "flags", 1, &data->flags);
  add_fieldinfo(Interface::IFT_BOOL, "visible", 1, &data->visible);
  add_fieldinfo(Interface::IFT_BOOL, "valid", 1, &data->valid);
  add_fieldinfo(Interface::IFT_INT, "visibility_history", 1, &data->visibility_history);
  add_fieldinfo(Interface::IFT_FLOAT, "roll", 1, &data->roll);
  add_fieldinfo(Interface::IFT_FLOAT, "pitch", 1, &data->pitch);
  add_fieldinfo(Interface::IFT_FLOAT, "yaw", 1, &data->yaw);
  add_fieldinfo(Interface::IFT_FLOAT, "distance", 1, &data->distance);
  add_fieldinfo(Interface::IFT_FLOAT, "bearing", 1, &data->bearing);
  add_fieldinfo(Interface::IFT_FLOAT, "slope", 1, &data->slope);
  add_fieldinfo(Interface::IFT_FLOAT, "dbs_covariance", 9, &data->dbs_covariance);
  add_fieldinfo(Interface::IFT_FLOAT, "world_x", 1, &data->world_x);
  add_fieldinfo(Interface::IFT_FLOAT, "world_y", 1, &data->world_y);
  add_fieldinfo(Interface::IFT_FLOAT, "world_z", 1, &data->world_z);
  add_fieldinfo(Interface::IFT_FLOAT, "world_xyz_covariance", 9, &data->world_xyz_covariance);
  add_fieldinfo(Interface::IFT_FLOAT, "relative_x", 1, &data->relative_x);
  add_fieldinfo(Interface::IFT_FLOAT, "relative_y", 1, &data->relative_y);
  add_fieldinfo(Interface::IFT_FLOAT, "relative_z", 1, &data->relative_z);
  add_fieldinfo(Interface::IFT_FLOAT, "relative_xyz_covariance", 9, &data->relative_xyz_covariance);
  add_fieldinfo(Interface::IFT_FLOAT, "extent_x", 1, &data->extent_x);
  add_fieldinfo(Interface::IFT_FLOAT, "extent_y", 1, &data->extent_y);
  add_fieldinfo(Interface::IFT_FLOAT, "extent_z", 1, &data->extent_z);
  add_fieldinfo(Interface::IFT_FLOAT, "world_x_velocity", 1, &data->world_x_velocity);
  add_fieldinfo(Interface::IFT_FLOAT, "world_y_velocity", 1, &data->world_y_velocity);
  add_fieldinfo(Interface::IFT_FLOAT, "world_z_velocity", 1, &data->world_z_velocity);
  add_fieldinfo(Interface::IFT_FLOAT, "world_xyz_velocity_covariance", 9, &data->world_xyz_velocity_covariance);
  add_fieldinfo(Interface::IFT_FLOAT, "relative_x_velocity", 1, &data->relative_x_velocity);
  add_fieldinfo(Interface::IFT_FLOAT, "relative_y_velocity", 1, &data->relative_y_velocity);
  add_fieldinfo(Interface::IFT_FLOAT, "relative_z_velocity", 1, &data->relative_z_velocity);
  add_fieldinfo(Interface::IFT_FLOAT, "relative_xyz_velocity_covariance", 9, &data->relative_xyz_velocity_covariance);
  unsigned char tmp_hash[] = {0xe6, 0x60, 0x23, 0x1b, 0xa4, 0x1a, 0x32, 0xb3, 0xed, 0x8e, 0xf8, 0x7f, 0x13, 0x1c, 0x73, 0x1f};
  set_hash(tmp_hash);
}

/** Destructor */
ObjectPositionInterface::~ObjectPositionInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get object_type value.
 * 
      Object type, use constants to define
    
 * @return object_type value
 */
unsigned int
ObjectPositionInterface::object_type() const
{
  return data->object_type;
}

/** Get maximum length of object_type value.
 * @return length of object_type value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_object_type() const
{
  return 1;
}

/** Set object_type value.
 * 
      Object type, use constants to define
    
 * @param new_object_type new object_type value
 */
void
ObjectPositionInterface::set_object_type(const unsigned int new_object_type)
{
  data->object_type = new_object_type;
}

/** Get flags value.
 * 
      Bit-wise concatenated fields of FLAG_* constants. Denotes features that the
      writer of this interfaces provides. Use a bit-wise OR to concatenate multiple
      flags, use a bit-wise AND to check if a flag has been set.
    
 * @return flags value
 */
unsigned int
ObjectPositionInterface::flags() const
{
  return data->flags;
}

/** Get maximum length of flags value.
 * @return length of flags value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_flags() const
{
  return 1;
}

/** Set flags value.
 * 
      Bit-wise concatenated fields of FLAG_* constants. Denotes features that the
      writer of this interfaces provides. Use a bit-wise OR to concatenate multiple
      flags, use a bit-wise AND to check if a flag has been set.
    
 * @param new_flags new flags value
 */
void
ObjectPositionInterface::set_flags(const unsigned int new_flags)
{
  data->flags = new_flags;
}

/** Get visible value.
 * True, if object is visible.
 * @return visible value
 */
bool
ObjectPositionInterface::is_visible() const
{
  return data->visible;
}

/** Get maximum length of visible value.
 * @return length of visible value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_visible() const
{
  return 1;
}

/** Set visible value.
 * True, if object is visible.
 * @param new_visible new visible value
 */
void
ObjectPositionInterface::set_visible(const bool new_visible)
{
  data->visible = new_visible;
}

/** Get valid value.
 * True, if this position is valid.
 * @return valid value
 */
bool
ObjectPositionInterface::is_valid() const
{
  return data->valid;
}

/** Get maximum length of valid value.
 * @return length of valid value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_valid() const
{
  return 1;
}

/** Set valid value.
 * True, if this position is valid.
 * @param new_valid new valid value
 */
void
ObjectPositionInterface::set_valid(const bool new_valid)
{
  data->valid = new_valid;
}

/** Get visibility_history value.
 * 
      The visibilitiy history indicates the number of consecutive positive or negative
      sightings. If the history is negative, there have been as many negative sightings
      (object not visible) as the absolute value of the history. A positive value denotes
      as many positive sightings. 0 shall only be used during the initialisation of the
      interface or if the visibility history is not filled.
    
 * @return visibility_history value
 */
int
ObjectPositionInterface::visibility_history() const
{
  return data->visibility_history;
}

/** Get maximum length of visibility_history value.
 * @return length of visibility_history value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_visibility_history() const
{
  return 1;
}

/** Set visibility_history value.
 * 
      The visibilitiy history indicates the number of consecutive positive or negative
      sightings. If the history is negative, there have been as many negative sightings
      (object not visible) as the absolute value of the history. A positive value denotes
      as many positive sightings. 0 shall only be used during the initialisation of the
      interface or if the visibility history is not filled.
    
 * @param new_visibility_history new visibility_history value
 */
void
ObjectPositionInterface::set_visibility_history(const int new_visibility_history)
{
  data->visibility_history = new_visibility_history;
}

/** Get roll value.
 * 
      Roll value for the orientation of the object in space.
    
 * @return roll value
 */
float
ObjectPositionInterface::roll() const
{
  return data->roll;
}

/** Get maximum length of roll value.
 * @return length of roll value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_roll() const
{
  return 1;
}

/** Set roll value.
 * 
      Roll value for the orientation of the object in space.
    
 * @param new_roll new roll value
 */
void
ObjectPositionInterface::set_roll(const float new_roll)
{
  data->roll = new_roll;
}

/** Get pitch value.
 * 
      Pitch value for the orientation of the object in space.
    
 * @return pitch value
 */
float
ObjectPositionInterface::pitch() const
{
  return data->pitch;
}

/** Get maximum length of pitch value.
 * @return length of pitch value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_pitch() const
{
  return 1;
}

/** Set pitch value.
 * 
      Pitch value for the orientation of the object in space.
    
 * @param new_pitch new pitch value
 */
void
ObjectPositionInterface::set_pitch(const float new_pitch)
{
  data->pitch = new_pitch;
}

/** Get yaw value.
 * 
      Yaw value for the orientation of the object in space.
    
 * @return yaw value
 */
float
ObjectPositionInterface::yaw() const
{
  return data->yaw;
}

/** Get maximum length of yaw value.
 * @return length of yaw value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_yaw() const
{
  return 1;
}

/** Set yaw value.
 * 
      Yaw value for the orientation of the object in space.
    
 * @param new_yaw new yaw value
 */
void
ObjectPositionInterface::set_yaw(const float new_yaw)
{
  data->yaw = new_yaw;
}

/** Get distance value.
 * 
      Distance from the robot to the object on the ground plane. The distance is given
      in meters.
    
 * @return distance value
 */
float
ObjectPositionInterface::distance() const
{
  return data->distance;
}

/** Get maximum length of distance value.
 * @return length of distance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_distance() const
{
  return 1;
}

/** Set distance value.
 * 
      Distance from the robot to the object on the ground plane. The distance is given
      in meters.
    
 * @param new_distance new distance value
 */
void
ObjectPositionInterface::set_distance(const float new_distance)
{
  data->distance = new_distance;
}

/** Get bearing value.
 * 
      Angle between the robot's forward direction and the object on the ground plane.
      This angle is in a local 3D coordinate system to the robot and given in radians.
    
 * @return bearing value
 */
float
ObjectPositionInterface::bearing() const
{
  return data->bearing;
}

/** Get maximum length of bearing value.
 * @return length of bearing value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_bearing() const
{
  return 1;
}

/** Set bearing value.
 * 
      Angle between the robot's forward direction and the object on the ground plane.
      This angle is in a local 3D coordinate system to the robot and given in radians.
    
 * @param new_bearing new bearing value
 */
void
ObjectPositionInterface::set_bearing(const float new_bearing)
{
  data->bearing = new_bearing;
}

/** Get slope value.
 * 
      Angle between the robot's center position on the ground plane and the middle point
      of the object (e.g. this denotes the height of the object combined with the distance.
      The angle is given in radians.
    
 * @return slope value
 */
float
ObjectPositionInterface::slope() const
{
  return data->slope;
}

/** Get maximum length of slope value.
 * @return length of slope value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_slope() const
{
  return 1;
}

/** Set slope value.
 * 
      Angle between the robot's center position on the ground plane and the middle point
      of the object (e.g. this denotes the height of the object combined with the distance.
      The angle is given in radians.
    
 * @param new_slope new slope value
 */
void
ObjectPositionInterface::set_slope(const float new_slope)
{
  data->slope = new_slope;
}

/** Get dbs_covariance value.
 * 
      Covariance of Distance/Yaw/Pitch values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @return dbs_covariance value
 */
float *
ObjectPositionInterface::dbs_covariance() const
{
  return data->dbs_covariance;
}

/** Get maximum length of dbs_covariance value.
 * @return length of dbs_covariance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_dbs_covariance() const
{
  return 9;
}

/** Set dbs_covariance value.
 * 
      Covariance of Distance/Yaw/Pitch values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @param new_dbs_covariance new dbs_covariance value
 */
void
ObjectPositionInterface::set_dbs_covariance(const float * new_dbs_covariance)
{
  memcpy(data->dbs_covariance, new_dbs_covariance, sizeof(float) * 9);
}

/** Get world_x value.
 * 
      This is the X coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
    
 * @return world_x value
 */
float
ObjectPositionInterface::world_x() const
{
  return data->world_x;
}

/** Get maximum length of world_x value.
 * @return length of world_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_world_x() const
{
  return 1;
}

/** Set world_x value.
 * 
      This is the X coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
    
 * @param new_world_x new world_x value
 */
void
ObjectPositionInterface::set_world_x(const float new_world_x)
{
  data->world_x = new_world_x;
}

/** Get world_y value.
 * 
      This is the Y coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right and Z pointing downwards.
    
 * @return world_y value
 */
float
ObjectPositionInterface::world_y() const
{
  return data->world_y;
}

/** Get maximum length of world_y value.
 * @return length of world_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_world_y() const
{
  return 1;
}

/** Set world_y value.
 * 
      This is the Y coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right and Z pointing downwards.
    
 * @param new_world_y new world_y value
 */
void
ObjectPositionInterface::set_world_y(const float new_world_y)
{
  data->world_y = new_world_y;
}

/** Get world_z value.
 * 
      This is the Z coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
    
 * @return world_z value
 */
float
ObjectPositionInterface::world_z() const
{
  return data->world_z;
}

/** Get maximum length of world_z value.
 * @return length of world_z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_world_z() const
{
  return 1;
}

/** Set world_z value.
 * 
      This is the Z coordinate in the cartesian right-handed world coordinate system.
      This coordinate system has its origin in the center of the field, Y pointing to
      the opponent's goal and X pointing to the right.
    
 * @param new_world_z new world_z value
 */
void
ObjectPositionInterface::set_world_z(const float new_world_z)
{
  data->world_z = new_world_z;
}

/** Get world_xyz_covariance value.
 * 
      Covariance of WorldX/WorldY/WorldZ values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @return world_xyz_covariance value
 */
float *
ObjectPositionInterface::world_xyz_covariance() const
{
  return data->world_xyz_covariance;
}

/** Get maximum length of world_xyz_covariance value.
 * @return length of world_xyz_covariance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_world_xyz_covariance() const
{
  return 9;
}

/** Set world_xyz_covariance value.
 * 
      Covariance of WorldX/WorldY/WorldZ values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @param new_world_xyz_covariance new world_xyz_covariance value
 */
void
ObjectPositionInterface::set_world_xyz_covariance(const float * new_world_xyz_covariance)
{
  memcpy(data->world_xyz_covariance, new_world_xyz_covariance, sizeof(float) * 9);
}

/** Get relative_x value.
 * 
      This is the X coordinate in the cartesian right-handed robot coordinate system.
    
 * @return relative_x value
 */
float
ObjectPositionInterface::relative_x() const
{
  return data->relative_x;
}

/** Get maximum length of relative_x value.
 * @return length of relative_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_relative_x() const
{
  return 1;
}

/** Set relative_x value.
 * 
      This is the X coordinate in the cartesian right-handed robot coordinate system.
    
 * @param new_relative_x new relative_x value
 */
void
ObjectPositionInterface::set_relative_x(const float new_relative_x)
{
  data->relative_x = new_relative_x;
}

/** Get relative_y value.
 * 
      This is the Y coordinate in the cartesian right-handed robot coordinate system.
    
 * @return relative_y value
 */
float
ObjectPositionInterface::relative_y() const
{
  return data->relative_y;
}

/** Get maximum length of relative_y value.
 * @return length of relative_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_relative_y() const
{
  return 1;
}

/** Set relative_y value.
 * 
      This is the Y coordinate in the cartesian right-handed robot coordinate system.
    
 * @param new_relative_y new relative_y value
 */
void
ObjectPositionInterface::set_relative_y(const float new_relative_y)
{
  data->relative_y = new_relative_y;
}

/** Get relative_z value.
 * 
      This is the Z coordinate in the cartesian right-handed robot coordinate system.
    
 * @return relative_z value
 */
float
ObjectPositionInterface::relative_z() const
{
  return data->relative_z;
}

/** Get maximum length of relative_z value.
 * @return length of relative_z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_relative_z() const
{
  return 1;
}

/** Set relative_z value.
 * 
      This is the Z coordinate in the cartesian right-handed robot coordinate system.
    
 * @param new_relative_z new relative_z value
 */
void
ObjectPositionInterface::set_relative_z(const float new_relative_z)
{
  data->relative_z = new_relative_z;
}

/** Get relative_xyz_covariance value.
 * 
      Covariance of relative x/y/z values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @return relative_xyz_covariance value
 */
float *
ObjectPositionInterface::relative_xyz_covariance() const
{
  return data->relative_xyz_covariance;
}

/** Get maximum length of relative_xyz_covariance value.
 * @return length of relative_xyz_covariance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_relative_xyz_covariance() const
{
  return 9;
}

/** Set relative_xyz_covariance value.
 * 
      Covariance of relative x/y/z values. This is a 3x3 matrix ordered line by line,
      first three values represent row, next tree values second row and last three values
      last row from left to right each.
    
 * @param new_relative_xyz_covariance new relative_xyz_covariance value
 */
void
ObjectPositionInterface::set_relative_xyz_covariance(const float * new_relative_xyz_covariance)
{
  memcpy(data->relative_xyz_covariance, new_relative_xyz_covariance, sizeof(float) * 9);
}

/** Get extent_x value.
 * 
      Extent of the seen object given in the relative x cartesian coordinate in m.
    
 * @return extent_x value
 */
float
ObjectPositionInterface::extent_x() const
{
  return data->extent_x;
}

/** Get maximum length of extent_x value.
 * @return length of extent_x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_extent_x() const
{
  return 1;
}

/** Set extent_x value.
 * 
      Extent of the seen object given in the relative x cartesian coordinate in m.
    
 * @param new_extent_x new extent_x value
 */
void
ObjectPositionInterface::set_extent_x(const float new_extent_x)
{
  data->extent_x = new_extent_x;
}

/** Get extent_y value.
 * 
      Extent of the seen object given in the relative y cartesian coordinate in m.
    
 * @return extent_y value
 */
float
ObjectPositionInterface::extent_y() const
{
  return data->extent_y;
}

/** Get maximum length of extent_y value.
 * @return length of extent_y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_extent_y() const
{
  return 1;
}

/** Set extent_y value.
 * 
      Extent of the seen object given in the relative y cartesian coordinate in m.
    
 * @param new_extent_y new extent_y value
 */
void
ObjectPositionInterface::set_extent_y(const float new_extent_y)
{
  data->extent_y = new_extent_y;
}

/** Get extent_z value.
 * 
      Extent of the seen object given in the relative z cartesian coordinate in m.
    
 * @return extent_z value
 */
float
ObjectPositionInterface::extent_z() const
{
  return data->extent_z;
}

/** Get maximum length of extent_z value.
 * @return length of extent_z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_extent_z() const
{
  return 1;
}

/** Set extent_z value.
 * 
      Extent of the seen object given in the relative z cartesian coordinate in m.
    
 * @param new_extent_z new extent_z value
 */
void
ObjectPositionInterface::set_extent_z(const float new_extent_z)
{
  data->extent_z = new_extent_z;
}

/** Get world_x_velocity value.
 * 
      Velocity of object in the world coordinate system in X-direction in meter per second.
    
 * @return world_x_velocity value
 */
float
ObjectPositionInterface::world_x_velocity() const
{
  return data->world_x_velocity;
}

/** Get maximum length of world_x_velocity value.
 * @return length of world_x_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_world_x_velocity() const
{
  return 1;
}

/** Set world_x_velocity value.
 * 
      Velocity of object in the world coordinate system in X-direction in meter per second.
    
 * @param new_world_x_velocity new world_x_velocity value
 */
void
ObjectPositionInterface::set_world_x_velocity(const float new_world_x_velocity)
{
  data->world_x_velocity = new_world_x_velocity;
}

/** Get world_y_velocity value.
 * 
      Velocity of object in the world coordinate system in Y-direction in meter per second.
    
 * @return world_y_velocity value
 */
float
ObjectPositionInterface::world_y_velocity() const
{
  return data->world_y_velocity;
}

/** Get maximum length of world_y_velocity value.
 * @return length of world_y_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_world_y_velocity() const
{
  return 1;
}

/** Set world_y_velocity value.
 * 
      Velocity of object in the world coordinate system in Y-direction in meter per second.
    
 * @param new_world_y_velocity new world_y_velocity value
 */
void
ObjectPositionInterface::set_world_y_velocity(const float new_world_y_velocity)
{
  data->world_y_velocity = new_world_y_velocity;
}

/** Get world_z_velocity value.
 * 
      Velocity of object in the world coordinate system in Z-direction in meter per second.
    
 * @return world_z_velocity value
 */
float
ObjectPositionInterface::world_z_velocity() const
{
  return data->world_z_velocity;
}

/** Get maximum length of world_z_velocity value.
 * @return length of world_z_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_world_z_velocity() const
{
  return 1;
}

/** Set world_z_velocity value.
 * 
      Velocity of object in the world coordinate system in Z-direction in meter per second.
    
 * @param new_world_z_velocity new world_z_velocity value
 */
void
ObjectPositionInterface::set_world_z_velocity(const float new_world_z_velocity)
{
  data->world_z_velocity = new_world_z_velocity;
}

/** Get world_xyz_velocity_covariance value.
 * 
      Covariance of WorldX/WorldY/WorldZ velocity values. This is a 3x3 matrix ordered line
      by line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
    
 * @return world_xyz_velocity_covariance value
 */
float *
ObjectPositionInterface::world_xyz_velocity_covariance() const
{
  return data->world_xyz_velocity_covariance;
}

/** Get maximum length of world_xyz_velocity_covariance value.
 * @return length of world_xyz_velocity_covariance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_world_xyz_velocity_covariance() const
{
  return 9;
}

/** Set world_xyz_velocity_covariance value.
 * 
      Covariance of WorldX/WorldY/WorldZ velocity values. This is a 3x3 matrix ordered line
      by line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
    
 * @param new_world_xyz_velocity_covariance new world_xyz_velocity_covariance value
 */
void
ObjectPositionInterface::set_world_xyz_velocity_covariance(const float * new_world_xyz_velocity_covariance)
{
  memcpy(data->world_xyz_velocity_covariance, new_world_xyz_velocity_covariance, sizeof(float) * 9);
}

/** Get relative_x_velocity value.
 * 
      Velocity of object in the world coordinate system in X-direction in meter per second.
    
 * @return relative_x_velocity value
 */
float
ObjectPositionInterface::relative_x_velocity() const
{
  return data->relative_x_velocity;
}

/** Get maximum length of relative_x_velocity value.
 * @return length of relative_x_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_relative_x_velocity() const
{
  return 1;
}

/** Set relative_x_velocity value.
 * 
      Velocity of object in the world coordinate system in X-direction in meter per second.
    
 * @param new_relative_x_velocity new relative_x_velocity value
 */
void
ObjectPositionInterface::set_relative_x_velocity(const float new_relative_x_velocity)
{
  data->relative_x_velocity = new_relative_x_velocity;
}

/** Get relative_y_velocity value.
 * 
      Velocity of object in the world coordinate system in Y-direction in meter per second.
    
 * @return relative_y_velocity value
 */
float
ObjectPositionInterface::relative_y_velocity() const
{
  return data->relative_y_velocity;
}

/** Get maximum length of relative_y_velocity value.
 * @return length of relative_y_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_relative_y_velocity() const
{
  return 1;
}

/** Set relative_y_velocity value.
 * 
      Velocity of object in the world coordinate system in Y-direction in meter per second.
    
 * @param new_relative_y_velocity new relative_y_velocity value
 */
void
ObjectPositionInterface::set_relative_y_velocity(const float new_relative_y_velocity)
{
  data->relative_y_velocity = new_relative_y_velocity;
}

/** Get relative_z_velocity value.
 * 
      Velocity of object in the world coordinate system in Z-direction in meter per second.
    
 * @return relative_z_velocity value
 */
float
ObjectPositionInterface::relative_z_velocity() const
{
  return data->relative_z_velocity;
}

/** Get maximum length of relative_z_velocity value.
 * @return length of relative_z_velocity value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_relative_z_velocity() const
{
  return 1;
}

/** Set relative_z_velocity value.
 * 
      Velocity of object in the world coordinate system in Z-direction in meter per second.
    
 * @param new_relative_z_velocity new relative_z_velocity value
 */
void
ObjectPositionInterface::set_relative_z_velocity(const float new_relative_z_velocity)
{
  data->relative_z_velocity = new_relative_z_velocity;
}

/** Get relative_xyz_velocity_covariance value.
 * 
      Covariance of relative x/y/z velocity values. This is a 3x3 matrix ordered line
      by line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
    
 * @return relative_xyz_velocity_covariance value
 */
float *
ObjectPositionInterface::relative_xyz_velocity_covariance() const
{
  return data->relative_xyz_velocity_covariance;
}

/** Get maximum length of relative_xyz_velocity_covariance value.
 * @return length of relative_xyz_velocity_covariance value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectPositionInterface::maxlenof_relative_xyz_velocity_covariance() const
{
  return 9;
}

/** Set relative_xyz_velocity_covariance value.
 * 
      Covariance of relative x/y/z velocity values. This is a 3x3 matrix ordered line
      by line, first three values represent row, next tree values second row and last three
      values last row from left to right each.
    
 * @param new_relative_xyz_velocity_covariance new relative_xyz_velocity_covariance value
 */
void
ObjectPositionInterface::set_relative_xyz_velocity_covariance(const float * new_relative_xyz_velocity_covariance)
{
  memcpy(data->relative_xyz_velocity_covariance, new_relative_xyz_velocity_covariance, sizeof(float) * 9);
}

/* =========== message create =========== */
Message *
ObjectPositionInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 */
bool
ObjectPositionInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(ObjectPositionInterface)
/// @endcond


} // end namespace fawkes
