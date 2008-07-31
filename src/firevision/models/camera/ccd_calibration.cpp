/***************************************************************************
 *  ccd_calibration.cpp - Class defining a ccd camera calibration matrix K
 *
 *  Generated: Thu May 8 13:53 2008
 *  Copyright  2008  Christof Rath <c.rath@student.tugraz.at>
 *
 *  $Id$
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


#include "ccd_calibration.h"

/** @class CCDCalibration <models/camera/ccd_calibration.h>
 * A Calibration matrix for a ccd camera
 * @author Christof Rath
 */

/**Constructor.
 * @param ax is the scale factor in the x-coordinate direction
 * @param ay is the scale factor in the y-coordinate direction
 * @param x0 is the x-coordinate of the principal point
 * @param y0 is the y-coordinate of the principal point
 */
CCDCalibration::CCDCalibration(const float ax, const float ay, const float x0, const float y0): Calibration()
{
  Matrix k(3, 3);
  k.id();
  k(0, 0) = ax;
  k(0, 2) = x0;
  k(1, 1) = ay;
  k(1, 2) = y0;

  K(k);
}

/** Copy constructor.
 * @param cp the CCDCalibration to copy
 */
CCDCalibration::CCDCalibration(const CCDCalibration& cp): Calibration()
{
  K(cp.K());
}

/** Destructor.
 */
CCDCalibration::~CCDCalibration()
{
}