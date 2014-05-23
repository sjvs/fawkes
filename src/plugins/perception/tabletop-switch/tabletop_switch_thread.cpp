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
TabletopSwitchThread::loop()
{

  bool last_switch_state = switch_if_->is_enabled();
  while (! switch_if_->msgq_empty()) {
    if (SwitchInterface::EnableSwitchMessage *msg =
        switch_if_->msgq_first_safe(msg))
    {
      switch_if_->set_enabled(true);
      switch_if_->write();
    } else if (SwitchInterface::DisableSwitchMessage *msg =
        switch_if_->msgq_first_safe(msg))
    {
      switch_if_->set_enabled(false);
      switch_if_->write();
    }

    switch_if_->msgq_pop();
  }

  if (last_switch_state != switch_if_->is_enabled()) {
    for (list<SwitchInterface *>::iterator it = tabletop_switch_ifs_.begin(); it != tabletop_switch_ifs_.end(); it++) {
      Message *msg;
      if (switch_if_->is_enabled()) {
//        logger->log_debug(name(), "Enabling %s", (*it)->id());
        msg = new SwitchInterface::EnableSwitchMessage();
      } else {
//        logger->log_debug(name(), "Disabling %s", (*it)->id());
        msg = new SwitchInterface::DisableSwitchMessage();
      }
      (*it)->msgq_enqueue(msg);
    }
  }
}
