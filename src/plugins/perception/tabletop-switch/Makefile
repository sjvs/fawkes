#*****************************************************************************
#            Makefile Build System for Fawkes: Tabletop Switch Plugin
#                            -------------------
#   Created on Fri May 23 12:03:42 2014
#   Copyright (C) 2014 by Till Hofmann
#
#*****************************************************************************
#
#   This program is free software; you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation; either version 2 of the License, or
#   (at your option) any later version.
#
#*****************************************************************************


BASEDIR = ../../../..

include $(BASEDIR)/etc/buildsys/config.mk

LIBS_tabletop_switch = fawkescore fawkesutils fawkesaspects fvutils \
	       fawkestf fawkesinterface fawkesblackboard \
	       SwitchInterface
OBJS_tabletop_switch = tabletop_switch_plugin.o tabletop_switch_thread.o

OBJS_all    = $(OBJS_tabletop_switch)

PLUGINS_all = $(PLUGINDIR)/tabletop-switch.$(SOEXT)

include $(BUILDSYSDIR)/base.mk
