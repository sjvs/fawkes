
/***************************************************************************
 *  TabletopEdgesInterface.cpp - Fawkes BlackBoard Interface - TabletopEdgesInterface
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

#include <interfaces/TabletopEdgesInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class TabletopEdgesInterface <interfaces/TabletopEdgesInterface.h>
 * TabletopEdgesInterface Fawkes BlackBoard Interface.
 * 
      Storage for the edges of a table. Used for visualization.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
TabletopEdgesInterface::TabletopEdgesInterface() : Interface()
{
  data_size = sizeof(TabletopEdgesInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (TabletopEdgesInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "frame", 32, data->frame);
  add_fieldinfo(IFT_UINT32, "num_points", 1, &data->num_points);
  add_fieldinfo(IFT_DOUBLE, "x", 20, &data->x);
  add_fieldinfo(IFT_DOUBLE, "y", 20, &data->y);
  add_fieldinfo(IFT_DOUBLE, "z", 20, &data->z);
  add_fieldinfo(IFT_DOUBLE, "goodness", 20, &data->goodness);
  unsigned char tmp_hash[] = {0xc8, 0x44, 0xb, 0xf8, 0xa5, 0x2a, 0xea, 0xdb, 0xd8, 0xb9, 0x83, 0x28, 0x52, 0x9c, 0x46, 0xd3};
  set_hash(tmp_hash);
}

/** Destructor */
TabletopEdgesInterface::~TabletopEdgesInterface()
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
TabletopEdgesInterface::frame() const
{
  return data->frame;
}

/** Get maximum length of frame value.
 * @return length of frame value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TabletopEdgesInterface::maxlenof_frame() const
{
  return 32;
}

/** Set frame value.
 * 
      Reference coordinate frame for the data.
    
 * @param new_frame new frame value
 */
void
TabletopEdgesInterface::set_frame(const char * new_frame)
{
  strncpy(data->frame, new_frame, sizeof(data->frame));
  data_changed = true;
}

/** Get num_points value.
 * 
        Number of edges
    
 * @return num_points value
 */
uint32_t
TabletopEdgesInterface::num_points() const
{
  return data->num_points;
}

/** Get maximum length of num_points value.
 * @return length of num_points value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TabletopEdgesInterface::maxlenof_num_points() const
{
  return 1;
}

/** Set num_points value.
 * 
        Number of edges
    
 * @param new_num_points new num_points value
 */
void
TabletopEdgesInterface::set_num_points(const uint32_t new_num_points)
{
  data->num_points = new_num_points;
  data_changed = true;
}

/** Get x value.
 * 
      The x coordinate of an edge point
    
 * @return x value
 */
double *
TabletopEdgesInterface::x() const
{
  return data->x;
}

/** Get x value at given index.
 * 
      The x coordinate of an edge point
    
 * @param index index of value
 * @return x value
 * @exception Exception thrown if index is out of bounds
 */
double
TabletopEdgesInterface::x(unsigned int index) const
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
TabletopEdgesInterface::maxlenof_x() const
{
  return 20;
}

/** Set x value.
 * 
      The x coordinate of an edge point
    
 * @param new_x new x value
 */
void
TabletopEdgesInterface::set_x(const double * new_x)
{
  memcpy(data->x, new_x, sizeof(double) * 20);
  data_changed = true;
}

/** Set x value at given index.
 * 
      The x coordinate of an edge point
    
 * @param new_x new x value
 * @param index index for of the value
 */
void
TabletopEdgesInterface::set_x(unsigned int index, const double new_x)
{
  if (index > 20) {
    throw Exception("Index value %u out of bounds (0..20)", index);
  }
  data->x[index] = new_x;
  data_changed = true;
}
/** Get y value.
 * 
      The y coordinate of an edge point
    
 * @return y value
 */
double *
TabletopEdgesInterface::y() const
{
  return data->y;
}

/** Get y value at given index.
 * 
      The y coordinate of an edge point
    
 * @param index index of value
 * @return y value
 * @exception Exception thrown if index is out of bounds
 */
double
TabletopEdgesInterface::y(unsigned int index) const
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
TabletopEdgesInterface::maxlenof_y() const
{
  return 20;
}

/** Set y value.
 * 
      The y coordinate of an edge point
    
 * @param new_y new y value
 */
void
TabletopEdgesInterface::set_y(const double * new_y)
{
  memcpy(data->y, new_y, sizeof(double) * 20);
  data_changed = true;
}

/** Set y value at given index.
 * 
      The y coordinate of an edge point
    
 * @param new_y new y value
 * @param index index for of the value
 */
void
TabletopEdgesInterface::set_y(unsigned int index, const double new_y)
{
  if (index > 20) {
    throw Exception("Index value %u out of bounds (0..20)", index);
  }
  data->y[index] = new_y;
  data_changed = true;
}
/** Get z value.
 * 
      The z coordinate of an edge point
    
 * @return z value
 */
double *
TabletopEdgesInterface::z() const
{
  return data->z;
}

/** Get z value at given index.
 * 
      The z coordinate of an edge point
    
 * @param index index of value
 * @return z value
 * @exception Exception thrown if index is out of bounds
 */
double
TabletopEdgesInterface::z(unsigned int index) const
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
TabletopEdgesInterface::maxlenof_z() const
{
  return 20;
}

/** Set z value.
 * 
      The z coordinate of an edge point
    
 * @param new_z new z value
 */
void
TabletopEdgesInterface::set_z(const double * new_z)
{
  memcpy(data->z, new_z, sizeof(double) * 20);
  data_changed = true;
}

/** Set z value at given index.
 * 
      The z coordinate of an edge point
    
 * @param new_z new z value
 * @param index index for of the value
 */
void
TabletopEdgesInterface::set_z(unsigned int index, const double new_z)
{
  if (index > 20) {
    throw Exception("Index value %u out of bounds (0..20)", index);
  }
  data->z[index] = new_z;
  data_changed = true;
}
/** Get goodness value.
 * 
      The z coordinate of an edge point
    
 * @return goodness value
 */
double *
TabletopEdgesInterface::goodness() const
{
  return data->goodness;
}

/** Get goodness value at given index.
 * 
      The z coordinate of an edge point
    
 * @param index index of value
 * @return goodness value
 * @exception Exception thrown if index is out of bounds
 */
double
TabletopEdgesInterface::goodness(unsigned int index) const
{
  if (index > 20) {
    throw Exception("Index value %u out of bounds (0..20)", index);
  }
  return data->goodness[index];
}

/** Get maximum length of goodness value.
 * @return length of goodness value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
TabletopEdgesInterface::maxlenof_goodness() const
{
  return 20;
}

/** Set goodness value.
 * 
      The z coordinate of an edge point
    
 * @param new_goodness new goodness value
 */
void
TabletopEdgesInterface::set_goodness(const double * new_goodness)
{
  memcpy(data->goodness, new_goodness, sizeof(double) * 20);
  data_changed = true;
}

/** Set goodness value at given index.
 * 
      The z coordinate of an edge point
    
 * @param new_goodness new goodness value
 * @param index index for of the value
 */
void
TabletopEdgesInterface::set_goodness(unsigned int index, const double new_goodness)
{
  if (index > 20) {
    throw Exception("Index value %u out of bounds (0..20)", index);
  }
  data->goodness[index] = new_goodness;
  data_changed = true;
}
/* =========== message create =========== */
Message *
TabletopEdgesInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
TabletopEdgesInterface::copy_values(const Interface *other)
{
  const TabletopEdgesInterface *oi = dynamic_cast<const TabletopEdgesInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(TabletopEdgesInterface_data_t));
}

const char *
TabletopEdgesInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
TabletopEdgesInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(TabletopEdgesInterface)
/// @endcond


} // end namespace fawkes
