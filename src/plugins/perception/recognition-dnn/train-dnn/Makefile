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

BASEDIR = ../../../../..
include $(BASEDIR)/etc/buildsys/config.mk
include $(BUILDSYSDIR)/protobuf.mk

unexport ARCH
undefine ARCH

CFLAGS += $(CFLAGS_CPP11)
CFLAGS += -g -Wall -O0 -ffast-math -mavx -std=c++14
CFLAGS += -DCNN_SINGLE_THREAD -DCNN_USE_AVX -DBOOST_SYSTEM_NO_DEPRECATED -DBOOST_NO_CXX11_SCOPED_ENUMS
CFLAGS += -DDNN_USE_IMAGE_API -DUSE_PTHREAD=ON -DCNN_USE_CAFFE_CONVERTER -DNO_STRICT
CFLAGS := $(filter-out -DCNN_SINGLE_THREAD,$(CFLAGS))

LIBS_dnn_train = m z boost_system boost_iostreams boost_serialization boost_filesystem pthread protobuf
LDFLAGS += `pkg-config --cflags --libs opencv` -pthread

OBJS_dnn_train = dnn_train.o caffe.pb.o 

OBJS_all = $(OBJS_imagetrain)

ifdef TINY_DNN
	INCDIRS+=$(TINY_DNN)
else 
	WARN_TARGETS += warning_tiny_dnn	
	INCDIRS+=/home/sagre/caros/tiny-dnn/
endif

ifeq ($(HAVE_PROTOBUF),1)
	CFLAGS += $(CFLAGS_PROTOBUF)   
else
	WARN_TARGETS += warning_protobuf
endif

BINS_all += $(BINDIR)/dnn_train

ifeq ($(OBJSSUBMAKE),1)
all: $(WARN_TARGETS) 

.PHONY: warning_tiny_dnn
warning_tiny_dnn:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Tiny Dnn includes not found. Using default path src/libs/tiny_dnn/"
warning_protobuf:
	$(SILENT)echo -e "$(INDENT_PRINT)--> $(TRED)Omitting dnn_train bin$(TNORMAL) (protobuf not available)"

endif

include $(BUILDSYSDIR)/base.mk
