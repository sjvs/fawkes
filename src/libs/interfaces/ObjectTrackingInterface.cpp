
/***************************************************************************
 *  ObjectTrackingInterface.cpp - Fawkes BlackBoard Interface - ObjectTrackingInterface
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

#include <interfaces/ObjectTrackingInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class ObjectTrackingInterface <interfaces/ObjectTrackingInterface.h>
 * ObjectTrackingInterface Fawkes BlackBoard Interface.
 * 
      Information about tracking. Provide messages to control
      the behavior of the tracker.
    
 * @ingroup FawkesInterfaces
 */



/** Constructor */
ObjectTrackingInterface::ObjectTrackingInterface() : Interface()
{
  data_size = sizeof(ObjectTrackingInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (ObjectTrackingInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_UINT32, "num_names", 1, &data->num_names);
  add_messageinfo("SuggestIdMessage");
  unsigned char tmp_hash[] = {0x74, 0x3e, 0xdc, 0xa9, 0x78, 0xb4, 0x89, 0x2, 0x24, 0x3c, 0xad, 0xfe, 0x33, 0xa6, 0x94, 0xff};
  set_hash(tmp_hash);
}

/** Destructor */
ObjectTrackingInterface::~ObjectTrackingInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get num_names value.
 * 
      Number of names that can currently be used.
    
 * @return num_names value
 */
uint32_t
ObjectTrackingInterface::num_names() const
{
  return data->num_names;
}

/** Get maximum length of num_names value.
 * @return length of num_names value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectTrackingInterface::maxlenof_num_names() const
{
  return 1;
}

/** Set num_names value.
 * 
      Number of names that can currently be used.
    
 * @param new_num_names new num_names value
 */
void
ObjectTrackingInterface::set_num_names(const uint32_t new_num_names)
{
  data->num_names = new_num_names;
  data_changed = true;
}

/* =========== message create =========== */
Message *
ObjectTrackingInterface::create_message(const char *type) const
{
  if ( strncmp("SuggestIdMessage", type, __INTERFACE_MESSAGE_TYPE_SIZE) == 0 ) {
    return new SuggestIdMessage();
  } else {
    throw UnknownTypeException("The given type '%s' does not match any known "
                               "message type for this interface type.", type);
  }
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
ObjectTrackingInterface::copy_values(const Interface *other)
{
  const ObjectTrackingInterface *oi = dynamic_cast<const ObjectTrackingInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(ObjectTrackingInterface_data_t));
}

const char *
ObjectTrackingInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** @class ObjectTrackingInterface::SuggestIdMessage <interfaces/ObjectTrackingInterface.h>
 * SuggestIdMessage Fawkes BlackBoard Interface Message.
 * 
    
 */


/** Constructor with initial values.
 * @param ini_obj_id initial value for obj_id
 */
ObjectTrackingInterface::SuggestIdMessage::SuggestIdMessage(const uint32_t ini_obj_id) : Message("SuggestIdMessage")
{
  data_size = sizeof(SuggestIdMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SuggestIdMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  data->obj_id = ini_obj_id;
  add_fieldinfo(IFT_UINT32, "obj_id", 1, &data->obj_id);
}
/** Constructor */
ObjectTrackingInterface::SuggestIdMessage::SuggestIdMessage() : Message("SuggestIdMessage")
{
  data_size = sizeof(SuggestIdMessage_data_t);
  data_ptr  = malloc(data_size);
  memset(data_ptr, 0, data_size);
  data      = (SuggestIdMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
  add_fieldinfo(IFT_UINT32, "obj_id", 1, &data->obj_id);
}

/** Destructor */
ObjectTrackingInterface::SuggestIdMessage::~SuggestIdMessage()
{
  free(data_ptr);
}

/** Copy constructor.
 * @param m message to copy from
 */
ObjectTrackingInterface::SuggestIdMessage::SuggestIdMessage(const SuggestIdMessage *m) : Message("SuggestIdMessage")
{
  data_size = m->data_size;
  data_ptr  = malloc(data_size);
  memcpy(data_ptr, m->data_ptr, data_size);
  data      = (SuggestIdMessage_data_t *)data_ptr;
  data_ts   = (message_data_ts_t *)data_ptr;
}

/* Methods */
/** Get obj_id value.
 * 
      The id to suggest.
    
 * @return obj_id value
 */
uint32_t
ObjectTrackingInterface::SuggestIdMessage::obj_id() const
{
  return data->obj_id;
}

/** Get maximum length of obj_id value.
 * @return length of obj_id value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
ObjectTrackingInterface::SuggestIdMessage::maxlenof_obj_id() const
{
  return 1;
}

/** Set obj_id value.
 * 
      The id to suggest.
    
 * @param new_obj_id new obj_id value
 */
void
ObjectTrackingInterface::SuggestIdMessage::set_obj_id(const uint32_t new_obj_id)
{
  data->obj_id = new_obj_id;
}

/** Clone this message.
 * Produces a message of the same type as this message and copies the
 * data to the new message.
 * @return clone of this message
 */
Message *
ObjectTrackingInterface::SuggestIdMessage::clone() const
{
  return new ObjectTrackingInterface::SuggestIdMessage(this);
}
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
ObjectTrackingInterface::message_valid(const Message *message) const
{
  const SuggestIdMessage *m0 = dynamic_cast<const SuggestIdMessage *>(message);
  if ( m0 != NULL ) {
    return true;
  }
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(ObjectTrackingInterface)
/// @endcond


} // end namespace fawkes
