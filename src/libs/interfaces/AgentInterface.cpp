
/***************************************************************************
 *  AgentInterface.cpp - Fawkes BlackBoard Interface - AgentInterface
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

#include <interfaces/AgentInterface.h>

#include <core/exceptions/software.h>

#include <cstring>
#include <cstdlib>

namespace fawkes {

/** @class AgentInterface <interfaces/AgentInterface.h>
 * AgentInterface Fawkes BlackBoard Interface.
 * This interface provides information about the running agent.
 * @ingroup FawkesInterfaces
 */



/** Constructor */
AgentInterface::AgentInterface() : Interface()
{
  data_size = sizeof(AgentInterface_data_t);
  data_ptr  = malloc(data_size);
  data      = (AgentInterface_data_t *)data_ptr;
  data_ts   = (interface_data_ts_t *)data_ptr;
  memset(data_ptr, 0, data_size);
  add_fieldinfo(IFT_STRING, "history", 20000, data->history);
  add_fieldinfo(IFT_STRING, "plan", 20000, data->plan);
  add_fieldinfo(IFT_STRING, "message", 256, data->message);
  unsigned char tmp_hash[] = {0x6e, 0x4, 0x90, 0x50, 0xc8, 0xd6, 0xa8, 0x6d, 0x54, 0xf1, 0x9, 0x5d, 0x9d, 0xad, 0xb6, 0x16};
  set_hash(tmp_hash);
}

/** Destructor */
AgentInterface::~AgentInterface()
{
  free(data_ptr);
}
/* Methods */
/** Get history value.
 * All actions performed in the past.
 * @return history value
 */
char *
AgentInterface::history() const
{
  return data->history;
}

/** Get maximum length of history value.
 * @return length of history value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AgentInterface::maxlenof_history() const
{
  return 20000;
}

/** Set history value.
 * All actions performed in the past.
 * @param new_history new history value
 */
void
AgentInterface::set_history(const char * new_history)
{
  strncpy(data->history, new_history, sizeof(data->history));
  data_changed = true;
}

/** Get plan value.
 * All planned actions.
 * @return plan value
 */
char *
AgentInterface::plan() const
{
  return data->plan;
}

/** Get maximum length of plan value.
 * @return length of plan value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AgentInterface::maxlenof_plan() const
{
  return 20000;
}

/** Set plan value.
 * All planned actions.
 * @param new_plan new plan value
 */
void
AgentInterface::set_plan(const char * new_plan)
{
  strncpy(data->plan, new_plan, sizeof(data->plan));
  data_changed = true;
}

/** Get message value.
 * Info message.
 * @return message value
 */
char *
AgentInterface::message() const
{
  return data->message;
}

/** Get maximum length of message value.
 * @return length of message value, can be length of the array or number of 
 * maximum number of characters for a string
 */
size_t
AgentInterface::maxlenof_message() const
{
  return 256;
}

/** Set message value.
 * Info message.
 * @param new_message new message value
 */
void
AgentInterface::set_message(const char * new_message)
{
  strncpy(data->message, new_message, sizeof(data->message));
  data_changed = true;
}

/* =========== message create =========== */
Message *
AgentInterface::create_message(const char *type) const
{
  throw UnknownTypeException("The given type '%s' does not match any known "
                             "message type for this interface type.", type);
}


/** Copy values from other interface.
 * @param other other interface to copy values from
 */
void
AgentInterface::copy_values(const Interface *other)
{
  const AgentInterface *oi = dynamic_cast<const AgentInterface *>(other);
  if (oi == NULL) {
    throw TypeMismatchException("Can only copy values from interface of same type (%s vs. %s)",
                                type(), other->type());
  }
  memcpy(data, oi->data, sizeof(AgentInterface_data_t));
}

const char *
AgentInterface::enum_tostring(const char *enumtype, int val) const
{
  throw UnknownTypeException("Unknown enum type %s", enumtype);
}

/* =========== messages =========== */
/** Check if message is valid and can be enqueued.
 * @param message Message to check
 * @return true if the message is valid, false otherwise.
 */
bool
AgentInterface::message_valid(const Message *message) const
{
  return false;
}

/// @cond INTERNALS
EXPORT_INTERFACE(AgentInterface)
/// @endcond


} // end namespace fawkes
