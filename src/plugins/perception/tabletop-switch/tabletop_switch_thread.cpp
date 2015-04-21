/***************************************************************************
 *  tabletop_switch_thread.cpp - Plugin to switch on/off all tabletop plugins
 *
 *  Created: Fri May 23 12:05:42 2014
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

#include "tabletop_switch_thread.h"

#include <interfaces/SwitchInterface.h>

#include <string.h>

using namespace fawkes;
using namespace std;

#define CFG_PREFIX "/perception/tabletop-objects/"

/** @class TabletopSwitchThread "tabletop_switch_thread.h"
 * Thread to switch on/off tabletop plugins
 * @author Till Hofmann
 */

/** Constructor. */
TabletopSwitchThread::TabletopSwitchThread()
  : Thread("TabletopSwitchThread", Thread::OPMODE_WAITFORWAKEUP),
    BlockedTimingAspect(BlockedTimingAspect::WAKEUP_HOOK_PRE_LOOP)
{
}

void
TabletopSwitchThread::init()
{
  cfg_initial_state_ = true;
  try {
    cfg_initial_state_ = config->get_bool(CFG_PREFIX"initial_state");
  } catch (Exception &e) { } // ignore, use default
  try {
    switch_if_ = blackboard->open_for_writing<SwitchInterface>("tabletop-objects");
    tabletop_switch_ifs_ = blackboard->open_multiple_for_reading<SwitchInterface>("object*");

    list<SwitchInterface *> tabletop_ifs_ = blackboard->open_multiple_for_reading<SwitchInterface>("tabletop*");
    tabletop_switch_ifs_.splice(tabletop_switch_ifs_.end(), tabletop_ifs_);

    for (list<SwitchInterface *>::iterator it = tabletop_switch_ifs_.begin(); it != tabletop_switch_ifs_.end(); it++) {
      if (strcmp((*it)->id(), "tabletop-objects") == 0) {
        blackboard->close(*it);
        tabletop_switch_ifs_.erase(it);
        break;
      }
    }
  } catch (const Exception &e) {
    blackboard->close(switch_if_);
    for (list<SwitchInterface *>::iterator it = tabletop_switch_ifs_.begin(); it != tabletop_switch_ifs_.end(); it++) {
      blackboard->close(*it);
    }
  }
}

void
TabletopSwitchThread::finalize()
{
  blackboard->close(switch_if_);
  for (list<SwitchInterface *>::iterator it = tabletop_switch_ifs_.begin(); it != tabletop_switch_ifs_.end(); it++) {
    blackboard->close(*it);
  }
}


void
TabletopSwitchThread::once()
{
  logger->log_debug(name(), "Initializing all plugins to %d", cfg_initial_state_);
  msg_all_interfaces(cfg_initial_state_);
}

void
TabletopSwitchThread::loop()
{

  bool last_switch_state = switch_if_->is_enabled();
  bool new_state = last_switch_state;
  while (! switch_if_->msgq_empty()) {
    if (SwitchInterface::EnableSwitchMessage *msg =
        switch_if_->msgq_first_safe(msg))
    {
      new_state = true;
    } else if (SwitchInterface::DisableSwitchMessage *msg =
        switch_if_->msgq_first_safe(msg))
    {
      new_state = false;
    }

    switch_if_->msgq_pop();
  }

  if (last_switch_state != new_state) {
    msg_all_interfaces(new_state);
    switch_if_->set_enabled(new_state);
    switch_if_->write();
  }
}

void
TabletopSwitchThread::msg_all_interfaces(const bool new_state)
{
  for (list<SwitchInterface *>::iterator it = tabletop_switch_ifs_.begin(); it != tabletop_switch_ifs_.end(); it++) {
    Message *msg;
    if (new_state) {
      msg = new SwitchInterface::EnableSwitchMessage();
    } else {
      msg = new SwitchInterface::DisableSwitchMessage();
    }
    (*it)->msgq_enqueue(msg);
  }
}
