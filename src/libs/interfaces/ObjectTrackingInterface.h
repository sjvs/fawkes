
/***************************************************************************
 *  ObjectTrackingInterface.h - Fawkes BlackBoard Interface - ObjectTrackingInterface
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

#ifndef __INTERFACES_OBJECTTRACKINGINTERFACE_H_
#define __INTERFACES_OBJECTTRACKINGINTERFACE_H_

#include <interface/interface.h>
#include <interface/message.h>
#include <interface/field_iterator.h>

namespace fawkes {

class ObjectTrackingInterface : public Interface
{
 /// @cond INTERNALS
 INTERFACE_MGMT_FRIENDS(ObjectTrackingInterface)
 /// @endcond
 public:
  /* constants */

 private:
#pragma pack(push,4)
  /** Internal data storage, do NOT modify! */
  typedef struct {
    int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
    int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
    uint32_t num_names; /**< 
      Number of names that can currently be used.
     */
  } ObjectTrackingInterface_data_t;
#pragma pack(pop)

  ObjectTrackingInterface_data_t *data;

 public:
  /* messages */
  class SuggestIdMessage : public Message
  {
   private:
#pragma pack(push,4)
    /** Internal data storage, do NOT modify! */
    typedef struct {
      int64_t timestamp_sec;  /**< Interface Unix timestamp, seconds */
      int64_t timestamp_usec; /**< Interface Unix timestamp, micro-seconds */
      uint32_t obj_id; /**< 
      The id to suggest.
     */
    } SuggestIdMessage_data_t;
#pragma pack(pop)

    SuggestIdMessage_data_t *data;

   public:
    SuggestIdMessage(const uint32_t ini_obj_id);
    SuggestIdMessage();
    ~SuggestIdMessage();

    SuggestIdMessage(const SuggestIdMessage *m);
    /* Methods */
    uint32_t obj_id() const;
    void set_obj_id(const uint32_t new_obj_id);
    size_t maxlenof_obj_id() const;
    virtual Message * clone() const;
  };

  virtual bool message_valid(const Message *message) const;
 private:
  ObjectTrackingInterface();
  ~ObjectTrackingInterface();

 public:
  /* Methods */
  uint32_t num_names() const;
  void set_num_names(const uint32_t new_num_names);
  size_t maxlenof_num_names() const;
  virtual Message * create_message(const char *type) const;

  virtual void copy_values(const Interface *other);
  virtual const char * enum_tostring(const char *enumtype, int val) const;

};

} // end namespace fawkes

#endif
