/***************************************************************************
 *  clingo_control_manager.cpp - Clingo control manager
 *
 *  Created: Thu Oct 27 16:23:32 2016
 *  Copyright  2016  Björn Schäpers
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

#include "clingo_access.h"
#include "clingo_control_manager.h"

#include <core/exception.h>
#include <logging/logger.h>

namespace fawkes {
#if 0 /* just to make Emacs auto-indent happy */
}
#endif

/**
 * @class ClingoControlManager <plugins/asp/aspect/clingo_control_manager.h>
 * The Clingo Control Manager creates and maintains Clingo Controls..
 * @author Björn Schäpers
 */

/**
 * Constructor.
 */
ClingoControlManager::ClingoControlManager(void) : Log(nullptr)
{
	return;
}

/** Destructor. */
ClingoControlManager::~ClingoControlManager(void)
{
	return;
}

/**
 * @brief Sets the logger for all Clingo Controls.
 * @param[in] logger The logger.
 */
void ClingoControlManager::setLogger(Logger *logger)
{
	Log = logger;
	return;
}

/**
 * Create a new control. The control is registered internally under the specified name.
 * It must be destroyed when done with it. Only a single control can be created for a particular control name.
 * @param[in] ctrl_name The Name by which to register the control.
 * @param[in] log_component_name The Prefix for log entries. If empty it will be set to "Clingo".
 * @return A new plain Clingo Control.
 */
LockPtr<ClingoAccess> ClingoControlManager::create_control(const std::string& ctrl_name,
		const std::string& log_component_name)
{
	if ( Controls.count(ctrl_name) != 0 )
	{
		throw Exception("Clingo Control '%s' already exists!", ctrl_name.c_str());
	} //if ( Controls.count(ctrl_name) != 0 )

	Clingo::SymbolSpan s;
	LockPtr<ClingoAccess> ctrl(new ClingoAccess(Log, log_component_name));

	Controls.emplace(ctrl_name, ctrl);

	return ctrl;
}

/**
 * "Destroys" the named control. Only ever destroy controls which you have created yourself.
 * It will be unregistered, but live as long as there is a LockPtr reference to it.
 * @param[in] ctrl_name The name of the control to destroy.
 */
void
ClingoControlManager::destroy_control(const std::string& ctrl_name)
{
	Controls.erase(ctrl_name);
	return;
}

/**
 * Get map of controls.
 * @return The map from control name to control lock ptr.
 */
const std::unordered_map<std::string, LockPtr<ClingoAccess>>&
ClingoControlManager::controls(void) const
{
	return Controls;
}

} // end namespace fawkes