#*****************************************************************************
#         Makefile Build System for Fawkes: realsense Plugin
#                            -------------------
#   Created on Mon Jun 13 17:09:44 2016
#   Copyright (C) 2016 by Johannes Rothe
#                 2018 by Till Hofmann
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
include $(BUILDCONFDIR)/tf/tf.mk

PRESUBDIRS = interfaces


CFLAGS += $(CFLAGS_CPP11) $(CFLAGS_TF) -DDNN_USE_IMAGE_API
LIBS_dnn_recognition = m fawkescore fawkesutils fawkesaspects fawkesbaseapp \
                      fawkesblackboard fawkesinterface \
                      DnnRecognitionInterface

OBJS_dnn_recognition = dnn_recognition_plugin.o dnn_recognition_thread.o 
OBJS_all = $(OBJS_dnn_recognition)

ifeq ($(HAVE_TF),1)
  ifdef TINY_DNN
	INCDIRS+=$(TINY_DNN)
  else 
	WARN_TARGETS += warning_tiny_dnn	
	INCDIRS+=$(HOME)/tiny-dnn
  endif
  PLUGINS_all = $(PLUGINDIR)/dnn_recognition.$(SOEXT)
else
	WARN_TARGETS += warning_tf
endif

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS)

.PHONY: warning_tiny_dnn warning_tf
warning_tiny_dnn:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Tiny Dnn includes not found. Using default path ~/tiny-dnn"
warning_tf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting genesis libs" \
	       		"$(TNORMAL) (tf not available)"
endif


include $(BUILDSYSDIR)/base.mk
