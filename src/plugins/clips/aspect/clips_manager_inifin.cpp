
/***************************************************************************
 *  clips_manager_inifin.cpp - CLIPSManagerAspect initializer/finalizer
 *
 *  Created: Fri Aug 16 16:06:45 2013
 *  Copyright  2006-2013  Tim Niemueller [www.niemueller.de]
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

#include <plugins/clips/aspect/clips_manager_inifin.h>
#include <plugins/clips/aspect/clips_env_manager.h>
#include <core/threading/thread_finalizer.h>

#include <clipsmm.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/** @class CLIPSManagerAspectIniFin <plugins/clips/aspect/clips_feature_inifin.h>
 * CLIPSManagerAspect initializer/finalizer.
 * @author Tim Niemueller
 */

/** Constructor. */
CLIPSManagerAspectIniFin::CLIPSManagerAspectIniFin()
  : AspectIniFin("CLIPSManagerAspect")
{
}

/** Destructor. */
CLIPSManagerAspectIniFin::~CLIPSManagerAspectIniFin()
{
}



void
CLIPSManagerAspectIniFin::init(Thread *thread)
{
  CLIPSManagerAspect *clips_thread;
  clips_thread = dynamic_cast<CLIPSManagerAspect *>(thread);
  if (clips_thread == NULL) {
    throw CannotInitializeThreadException("Thread '%s' claims to have the "
					  "CLIPSManagerAspect, but RTTI says it "
					  "has not. ", thread->name());
  }

  clips_thread->clips_env_mgr = clips_env_mgr_;
}


void
CLIPSManagerAspectIniFin::finalize(Thread *thread)
{
  CLIPSManagerAspect *clips_thread;
  clips_thread = dynamic_cast<CLIPSManagerAspect *>(thread);
  if (clips_thread == NULL) {
    throw CannotFinalizeThreadException("Thread '%s' claims to have the "
					"CLIPSManagerAspect, but RTTI says it "
					"has not. ", thread->name());
  }

  clips_thread->clips_env_mgr.clear();
}



/** Set CLIPS environment manger.
 * @param clips_env_mgr CLIPS environment manager
 */
void
CLIPSManagerAspectIniFin::set_manager(LockPtr<CLIPSEnvManager> &clips_env_mgr)
{
  clips_env_mgr_ = clips_env_mgr;
}

} // end namespace fawkes
