
/***************************************************************************
 *  TabletopHullInterface.cpp - Fawkes BlackBoard Interface - TabletopHullInterface
 *
 *  Templated created:   Thu Oct 12 10:49:19 2006
 *  Copyright  2014  Till Hofmann
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

#include <interfaces/TabletopHullInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class TabletopHullInterface <interfaces/TabletopHullInterface.h>
 * TabletopHullInterface Fawkes BlackBoard Interface.
 * 
      Storage for the hull vertices of a table. Used for visualization.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
TabletopHullInterface::TabletopHullInterface() : Interface()
{
  data_size = sizeof(TabletopHullInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (TabletopHullInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "frame", 32, data->frame);
  add_fieldinfo(IFT_UINT32, "num_points", 1, &data->num_points);
  add_fieldinfo(IFT_DOUBLE, "x", 20, &data->x);
  add_fieldinfo(IFT_DOUBLE, "y", 20, &data->y);
  add_fieldinfo(IFT_DOUBLE, "z", 20, &data->z);
  unsigned char tmp_hash[] = {0x4e, 00, 0x41, 0x10, 0x9c, 0xa2, 0x64, 0x1, 0xa0, 0x81, 0x66, 0x77, 0x4f, 0xca, 0xa0, 0x51};
  set_hash(tmp_hash);
}

/** Destructor */
TabletopHullInterface::~TabletopHullInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get frame value.
 * 
      Reference coordinate frame for the data.
    
 * @return frame value
 */
char *
TabletopHullInterface::frame() const
{
  return data->frame;
}

/** Get maximum length of frame value.
 * @return length of frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TabletopHullInterface::maxlenof_frame() const
{
  return 32;
}

/** Set frame value.
 * 
      Reference coordinate frame for the data.
    
 * @param new_frame new frame value
 */
void
TabletopHullInterface::set_frame(const char * new_frame)
{
  strncpy(data->frame, new_frame, sizeof(data->frame));
  data_changed = true;
}

/** Get num_points value.
 * 
        Number of points in the hull
    
 * @return num_points value
 */
uint32_t
TabletopHullInterface::num_points() const
{
  return data->num_points;
}

/** Get maximum length of num_points value.
 * @return length of num_points value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TabletopHullInterface::maxlenof_num_points() const
{
  return 1;
}

/** Set num_points value.
 * 
        Number of points in the hull
    
 * @param new_num_points new num_points value
 */
void
TabletopHullInterface::set_num_points(const uint32_t new_num_points)
{
  data->num_points = new_num_points;
  data_changed = true;
}

/** Get x value.
 * 
      The x coordinate of all hull vertices
    
 * @return x value
 */
double *
TabletopHullInterface::x() const
{
  return data->x;
}

/** Get x value at given index.
 * 
      The x coordinate of all hull vertices
    
 * @param index index of value
 * @return x value
 * @exception Exception thrown if index is out of bounds
 */
double
TabletopHullInterface::x(unsigned int index) const
{
  if (index > 20) {
    throw Exception("Index value %u out of bounds (0..20)", index);
  }
  return data->x[index];
}

/** Get maximum length of x value.
 * @return length of x value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TabletopHullInterface::maxlenof_x() const
{
  return 20;
}

/** Set x value.
 * 
      The x coordinate of all hull vertices
    
 * @param new_x new x value
 */
void
TabletopHullInterface::set_x(const double * new_x)
{
  memcpy(data->x, new_x, sizeof(double) * 20);
  data_changed = true;
}

/** Set x value at given index.
 * 
      The x coordinate of all hull vertices
    
 * @param new_x new x value
 * @param index index for of the value
 */
void
TabletopHullInterface::set_x(unsigned int index, const double new_x)
{
  if (index > 20) {
    throw Exception("Index value %u out of bounds (0..20)", index);
  }
  data->x[index] = new_x;
  data_changed = true;
}
/** Get y value.
 * 
      The y coordinate of all hull vertices
    
 * @return y value
 */
double *
TabletopHullInterface::y() const
{
  return data->y;
}

/** Get y value at given index.
 * 
      The y coordinate of all hull vertices
    
 * @param index index of value
 * @return y value
 * @exception Exception thrown if index is out of bounds
 */
double
TabletopHullInterface::y(unsigned int index) const
{
  if (index > 20) {
    throw Exception("Index value %u out of bounds (0..20)", index);
  }
  return data->y[index];
}

/** Get maximum length of y value.
 * @return length of y value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TabletopHullInterface::maxlenof_y() const
{
  return 20;
}

/** Set y value.
 * 
      The y coordinate of all hull vertices
    
 * @param new_y new y value
 */
void
TabletopHullInterface::set_y(const double * new_y)
{
  memcpy(data->y, new_y, sizeof(double) * 20);
  data_changed = true;
}

/** Set y value at given index.
 * 
      The y coordinate of all hull vertices
    
 * @param new_y new y value
 * @param index index for of the value
 */
void
TabletopHullInterface::set_y(unsigned int index, const double new_y)
{
  if (index > 20) {
    throw Exception("Index value %u out of bounds (0..20)", index);
  }
  data->y[index] = new_y;
  data_changed = true;
}
/** Get z value.
 * 
      The z coordinate of all hull vertices
    
 * @return z value
 */
double *
TabletopHullInterface::z() const
{
  return data->z;
}

/** Get z value at given index.
 * 
      The z coordinate of all hull vertices
    
 * @param index index of value
 * @return z value
 * @exception Exception thrown if index is out of bounds
 */
double
TabletopHullInterface::z(unsigned int index) const
{
  if (index > 20) {
    throw Exception("Index value %u out of bounds (0..20)", index);
  }
  return data->z[index];
}

/** Get maximum length of z value.
 * @return length of z value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TabletopHullInterface::maxlenof_z() const
{
  return 20;
}

/** Set z value.
 * 
      The z coordinate of all hull vertices
    
 * @param new_z new z value
 */
void
TabletopHullInterface::set_z(const double * new_z)
{
  memcpy(data->z, new_z, sizeof(double) * 20);
  data_changed = true;
}

/** Set z value at given index.
 * 
      The z coordinate of all hull vertices
    
 * @param new_z new z value
 * @param index index for of the value
 */
void
TabletopHullInterface::set_z(unsigned int index, const double new_z)
{
  if (index > 20) {
    throw Exception("Index value %u out of bounds (0..20)", index);
  }
  data->z[index] = new_z;
  data_changed = true;
}
/* =========== message create =========== */
Message *
TabletopHullInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
TabletopHullInterface::copy_values(const Interface *other)
{
  const TabletopHullInterface *oi = dynamic_cast<const TabletopHullInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(TabletopHullInterface_data_t));
}

const char *
TabletopHullInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
TabletopHullInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(TabletopHullInterface)
/// @endcond


} // end namespace fawkes
