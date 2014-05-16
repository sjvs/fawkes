#*****************************************************************************
#               Makefile Build System for Fawkes: Object Fitting 
#                            -------------------
#   Created on Tue Apr 29 13:03:02 2014
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
include $(BASEDIR)/src/plugins/perception/common/common.mk
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk
include $(BUILDSYSDIR)/pcl.mk

REQUIRED_PCL_LIBS = segmentation filters features

LIBS_object_fitting = stdc++ fawkescore fawkesutils fawkesaspects fvutils \
			fawkestf fawkesinterface fawkesblackboard fawkespcl_utils \
			Position3DInterface fawkesperception_common fawkessyncpoint \
			SwitchInterface

OBJS_object_fitting = object_fitting_plugin.o object_fitting_thread.o

OBJS_all = $(OBJS_object_fitting)

ifeq ($(HAVE_PCL)$(HAVE_TF),11)
  ifeq ($(call pcl-have-libs,$(REQUIRED_PCL_LIBS)),1)
    CFLAGS += $(CFLAGS_TF) $(CFLAGS_PCL) \
        $(call pcl-libs-cflags,$(REQUIRED_PCL_LIBS))
    LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_PCL) \
         $(call pcl-libs-ldflags,$(REQUIRED_PCL_LIBS))
    PLUGINS_all = $(PLUGINDIR)/object-fitting.$(SOEXT)
  else
    WARN_TARGETS += warning_pcl_components
  endif
else
  ifneq ($(HAVE_PCL),1)
    WARN_TARGETS += warning_pcl
  endif
  ifneq ($(HAVE_TF),1)
    WARN_TARGETS += warning_tf
  endif
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_pcl warning_pcl_components
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting object fitting plugin$(TNORMAL) (fawkestf not available)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting object fitting plugin$(TNORMAL) (PCL not available)"
warning_pcl_components:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting object fitting$(TNORMAL) (missing PCL components: $(call pcl-missing-libs,$(REQUIRED_PCL_LIBS)))"
endif


include $(BUILDSYSDIR)/base.mk
