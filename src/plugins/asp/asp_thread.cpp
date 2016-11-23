/***************************************************************************
 *  asp_thread.cpp - ASP environment providing Thread
 *
 *  Created: Thu Oct 20 15:49:31 2016
 *  Copyright  2016 Björn Schäpers
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

#include "asp_thread.h"

#include <clingo.hh>

using namespace fawkes;

/** @class ASPThread "clips_thread.h"
 * ASP environment thread.
 *
 * @author Björn Schäpers
 *
 * @property ASPThread::ASPIniFin
 * @brief The initi-/finalizer for the ASPAspect.
 *
 * @property ASPThread::ClingoIniFin
 * @brief The initi-/finalizer for the ClingoManagerAspect.
 *
 * @property ASPThread::CtrlMgr
 * @brief The clingo control manager.
 */

/** Constructor. */
ASPThread::ASPThread(void)
  : Thread("ASPThread", Thread::OPMODE_WAITFORWAKEUP),
	AspectProviderAspect([this](void) {
		std::list<fawkes::AspectIniFin*> ret;
		ret.emplace_back(&ASPIniFin);
		ret.emplace_back(&ClingoIniFin);
		return ret;
	}()),
	CtrlMgr(new ClingoControlManager)
{
	return;
}

void
ASPThread::init(void)
{
	CtrlMgr->setLogger(logger);
	ASPIniFin.setControlManager(CtrlMgr);
	ClingoIniFin.setControlManager(CtrlMgr);
	return;
}

void
ASPThread::finalize(void)
{
	return;
}

void
ASPThread::loop(void)
{
	return;
}