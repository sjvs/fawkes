#*****************************************************************************
#        Makefile Build System for Fawkes: Perception common library
#                            -------------------
#   Created on Tue Apr 22 14:57:13 2014
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

include $(BUILDSYSDIR)/pcl.mk
include $(BUILDCONFDIR)/tf/tf.mk

ifeq ($(HAVE_PCL),1)
  ifeq ($(HAVE_TF),1)
    CFLAGS_PERCEPTION_COMMON += $(CFLAGS_TF) $(CFLAGS_PCL)
    LDFLAGS_PERCEPTION_COMMON += $(LDFLAGS_TF) $(LDFLAGS_PCL)
    HAVE_PERCEPTION_COMMON = 1
  endif
endif
