#*****************************************************************************
#            Makefile Build System for Fawkes: Object Tracking Plugin
#                            -------------------
#   Created on Wed Apr 23 12:58:07 2014
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
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDCONFDIR)/tf/tf.mk

REQUIRED_PCL_LIBS = registration

LIBS_object_tracking = stdc++ fawkescore fawkesutils fawkesaspects fvutils \
			fawkestf fawkesinterface fawkesblackboard \
			Position3DInterface SwitchInterface fawkesperception_common \
			fawkessyncpoint ObjectTrackingInterface

OBJS_object_tracking = object_tracking_plugin.o object_tracking_thread.o

OBJS_all = $(OBJS_object_tracking)

ifeq ($(HAVE_PERCEPTION_COMMON),1)
  ifeq ($(HAVE_TF),1)
    ifeq ($(HAVE_PCL),1)
      ifeq ($(call pcl-have-libs,$(REQUIRED_PCL_LIBS)),1)
        CFLAGS += $(CFLAGS_PERCEPTION_COMMON) $(CFLAGS_PCL) $(CFLAGS_TF) \
	        $(call pcl-libs-cflags,$(REQUIRED_PCL_LIBS))
        LDFLAGS += $(LDFLAGS_PERCEPTION_COMMON) $(LDFLAGS_PCL) $(LDFLAGS_TF) \
	         $(call pcl-libs-ldflags,$(REQUIRED_PCL_LIBS))
        PLUGINS_all += $(PLUGINDIR)/object-tracking.$(SOEXT)
      else
        WARN_TARGETS += warning_pcl_components
      endif
    else
      WARN_TARGETS += warning_pcl
    endif
  else
    WARN_TARGETS += warning_tf
  endif
else
  WARN_TARGETS += warning_perception_common
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_perception_common warning_tf warning_pcl warning_pcl_components
warning_perception_common:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting object tracking plugin$(TNORMAL) (perception common library not available)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting object tracking plugin$(TNORMAL) (fawkestf not available)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting object tracking plugin$(TNORMAL) (PCL not available)"
warning_pcl_components:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting object tracking plugin$(TNORMAL) (missing PCL components: $(call pcl-missing-libs,$(REQUIRED_PCL_LIBS)))"

endif

include $(BUILDSYSDIR)/base.mk
