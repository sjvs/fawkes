#*****************************************************************************
#        Makefile Build System for Fawkes: Perception Common Library
#                            -------------------
#   Created on Thu Apr 17 12:27:1047 2014
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

LIBS_libfawkesperception_common = stdc++ fawkescore fawkesutils \
			fawkestf fawkesinterface Position3DInterface
OBJS_libfawkesperception_common = perception_common.o
HDRS_libfawkesperception_common = perception_common.h

OBJS_all = $(OBJS_libfawkesperception_common)

ifeq ($(HAVE_PERCEPTION_COMMON),1)
  CFLAGS += $(CFLAGS_PERCEPTION_COMMON)
  LDFLAGS += $(LDFLAGS_PERCEPTION_COMMON)
  LIBS_all += $(LIBDIR)/libfawkesperception_common.so
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

.PHONY: warning_tf warning_pcl
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting perception common library$(TNORMAL) (fawkestf not available)"
warning_pcl:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting perception common library$(TNORMAL) (PCL not available)"
endif

include $(BUILDSYSDIR)/base.mk
