#*****************************************************************************
#          Makefile Build System for Fawkes: perception plugins
#                            -------------------
#   Created on Sun Jan 14 18:24:42 2007
#   Copyright (C) 2006-2011 by Tim Niemueller, AllemaniACs RoboCup Team
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************

BASEDIR = ../../..

include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/fvconf.mk

SUBDIRS = base fountain retriever tabletop-objects pcl-db \
	tabletop-detection object-recognition common tabletop-visualization \
	object-tracking object-fitting

tabletop-detection: common
object-recognition: common
object-tracking: common
object-fitting: common
tabletop-visualization: common
include $(BUILDSYSDIR)/rules.mk

