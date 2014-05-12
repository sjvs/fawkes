#*****************************************************************************
#         Makefile Build System for Fawkes: Tabletop Visualization Plugin
#                            -------------------
#   Created on Thu Apr 10 16:43:42 2014
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
include $(BUILDSYSDIR)/ros.mk
include $(BUILDSYSDIR)/pcl.mk
include $(BUILDCONFDIR)/tf/tf.mk

LIBS_tabletop_visualization = fawkescore fawkesutils fawkesaspects fvutils \
			fawkestf fawkesinterface fawkesblackboard \
			Position3DInterface fawkesperception_common fawkessyncpoint

OBJS_tabletop_visualization = visualization_plugin.o visualization_thread.o

ifeq ($(HAVE_PERCEPTION_COMMON),1)
  ifeq ($(HAVE_TF),1)
    ifeq ($(HAVE_ROS),1)
      ifeq ($(call ros-have-pkg,visualization_msgs),1)
        CFLAGS += -DHAVE_VISUAL_DEBUGGING $(CFLAGS_ROS) $(CFLAGS_TF) \
                     $(CFLAGS_EIGEN3) $(call ros-pkg-cflags,visualization_msgs) \
                     $(CFLAGS_PERCEPTION_COMMON)
        LDFLAGS += $(LDFLAGS_ROS) $(call ros-pkg-lflags,visualization_msgs) \
                     $(LDFLAGS_PERCEPTION_COMMON)

        # Link with qhull, required by PCL therefore safe to assume it's there
        LIBS_tabletop_visualization += fawkesrosaspect qhull

        PLUGINS_all += $(PLUGINDIR)/tabletop-visualization.$(SOEXT)
      else
        WARN_TARGETS += warning_visualization_msgs
      endif
    else
      WARN_TARGETS += warning_ros
    endif
  else
    WARN_TARGETS += warning_tf
  endif
else
  WARN_TARGETS += warning_perception_common
endif

OBJS_all    = $(OBJS_tabletop_visualization)


ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_perception_common warning_tf warning_ros warning_visualization_msgs
warning_perception_common:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tabletop visualization plugin$(TNORMAL) (perception common library not available)"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tabletop visualization plugin$(TNORMAL) (fawkestf not available)"
warning_ros:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tabletop visualization plugin$(TNORMAL) (ROS not available)"
warning_visualization_msgs:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting tabletop visualization plugin$(TNORMAL) (ROS package visualization_msgs not available)"

endif

include $(BUILDSYSDIR)/base.mk

