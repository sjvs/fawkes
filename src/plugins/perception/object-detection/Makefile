#*****************************************************************************
#         Makefile Build System for Fawkes: Object Detection Plugin
#                            -------------------
#   Created on Tue Apr 15 17:11:17 2014
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
include $(BUILDSYSDIR)/boost.mk
include $(BUILDCONFDIR)/tf/tf.mk

REQUIRED_PCL_LIBS = segmentation filters

ifeq ($(CC),clang)
  CFLAGS += -Wno-overloaded-virtual
endif
ifeq ($(CC),gcc)
  ifneq ($(call gcc_atleast_version,4,5),1)
    GCC_TOO_OLD=1
  endif
endif

# Enable for time measurements
#CFLAGS += -DUSE_TIMETRACKER

LIBS_object_detection = stdc++ fawkescore fawkesutils fawkesaspects fvutils \
			fawkestf fawkesinterface fawkesblackboard fawkespcl_utils \
			Position3DInterface SwitchInterface \
			fawkesperception_common
OBJS_object_detection = object_detection_plugin.o object_detection_thread.o

OBJS_all    = $(OBJS_object_detection)

ifneq ($(CC)$(GCC_TOO_OLD),gcc1)
  ifeq ($(HAVE_PERCEPTION_COMMON)$(HAVE_PCL)$(HAVE_TF),111)
    ifeq ($(call pcl-have-libs,$(REQUIRED_PCL_LIBS)),1)
      CFLAGS += $(CFLAGS_TF) $(CFLAGS_PCL) \
	        $(call pcl-libs-cflags,$(REQUIRED_PCL_LIBS)) \
	        -Wno-deprecated -D_FILE_OFFSET_BITS=64 -D_LARGE_FILES

      LDFLAGS += $(LDFLAGS_TF) $(LDFLAGS_PCL) \
	         $(call pcl-libs-ldflags,$(REQUIRED_PCL_LIBS))

      PLUGINS_all = $(PLUGINDIR)/object-detection.$(SOEXT)
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
    ifneq ($(HAVE_PERCEPTION_COMMON),1)
      WARN_TARGETS += warning_perception_common
    endif
  endif
else
  WARN_TARGETS += warning_old_gcc
endif

ifneq ($(ENABLE_VISUAL_DEBUGGING),1)
  CFLAGS  += $(call boost-lib-cflags,thread)
  LDFLAGS += $(call boost-lib-ldflags,thread)
endif

# Required for weirdo PCL (on Fedora with ROS headers)
CFLAGS  += $(call boost-lib-cflags,system)
LDFLAGS += $(call boost-lib-ldflags,system)



ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tf warning_pcl warning_old_gcc warning_pcl_components warning_perception_common
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting object detection plugin$(TNORMAL) (fawkestf not available)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting object detection plugin$(TNORMAL) (PCL not available)"
warning_old_gcc:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting object detection plugin$(TNORMAL) (GCC too old, have $(GCC_VERSION), required 4.5)"
warning_pcl_components:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting object detection plugin$(TNORMAL) (missing PCL components: $(call pcl-missing-libs,$(REQUIRED_PCL_LIBS)))"
warning_perception_common:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting object detection plugin$(TNORMAL) (perception common library not available)"

endif

include $(BUILDSYSDIR)/base.mk

