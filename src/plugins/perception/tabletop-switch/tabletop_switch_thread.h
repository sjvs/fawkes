/***************************************************************************
 *  tabletop_switch_thread.h - Plugin to switch on/off all tabletop plugins
 *
 *  Created:  Fri May 23 11:58:42 2014
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

#ifndef __PLUGINS_PERCEPTION_TABLETOP_SWITCH_THREAD_H_
#define __PLUGINS_PERCEPTION_TABLETOP_SWITCH_THREAD_H_

#include <core/threading/thread.h>
#include <aspect/clock.h>
#include <aspect/blackboard.h>
#include <aspect/blocked_timing.h>
#include <aspect/logging.h>
#include <interfaces/SwitchInterface.h>

#include <list>

namespace fawkes {
  class SwitchInterface;
}

class TabletopSwitchThread
: public fawkes::Thread,
  public fawkes::ClockAspect,
  public fawkes::BlackBoardAspect,
  public fawkes::LoggingAspect,
  public fawkes::BlockedTimingAspect
{
 public:
  TabletopSwitchThread();

  virtual void init();
  virtual void loop();
  virtual void finalize();
 private:
  fawkes::SwitchInterface *switch_if_;
  std::list<fawkes::SwitchInterface *> tabletop_switch_ifs_;
  uint loopcount_;
};

#endif
