
/***************************************************************************
 *  TabletopEdgesInterface.h - Fawkes BlackBoard Interface - TabletopEdgesInterface
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

#ifndef __INTERFACES_TABLETOPEDGESINTERFACE_H_
#define __INTERFACES_TABLETOPEDGESINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class TabletopEdgesInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(TabletopEdgesInterface)
 /// @endcond
 public:
  /* constants */

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    char frame[32]; /**< 
      Reference coordinate frame for the data.
     */
    uint32_t num_points; /**< 
        Number of edges
     */
    double x[20]; /**< 
      The x coordinate of an edge point
     */
    double y[20]; /**< 
      The y coordinate of an edge point
     */
    double z[20]; /**< 
      The z coordinate of an edge point
     */
    double goodness[20]; /**< 
      The z coordinate of an edge point
     */
  } TabletopEdgesInterface_data_t;
#pragma pack(pop)

  TabletopEdgesInterface_data_t *data;

 public:
  /* messages */
  virtual bool message_valid(const Message *message) const;
 private:
  TabletopEdgesInterface();
  ~TabletopEdgesInterface();

 public:
  /* Methods */
  char * frame() const;
  void set_frame(const char * new_frame);
  size_t maxlenof_frame() const;
  uint32_t num_points() const;
  void set_num_points(const uint32_t new_num_points);
  size_t maxlenof_num_points() const;
  double * x() const;
  double x(unsigned int index) const;
  void set_x(unsigned int index, const double new_x);
  void set_x(const double * new_x);
  size_t maxlenof_x() const;
  double * y() const;
  double y(unsigned int index) const;
  void set_y(unsigned int index, const double new_y);
  void set_y(const double * new_y);
  size_t maxlenof_y() const;
  double * z() const;
  double z(unsigned int index) const;
  void set_z(unsigned int index, const double new_z);
  void set_z(const double * new_z);
  size_t maxlenof_z() const;
  double * goodness() const;
  double goodness(unsigned int index) const;
  void set_goodness(unsigned int index, const double new_goodness);
  void set_goodness(const double * new_goodness);
  size_t maxlenof_goodness() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
