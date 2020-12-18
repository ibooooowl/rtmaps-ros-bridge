# RTMaps Makefile
# Copyright 2002-2011 (C) Intempora S.A.

ROOTDIR  := $(realpath $(CURDIR)/..)
ifeq ($(ROOTDIR),)
$(error Please use make at least version 3.81)
endif

# Add C++ sources here
sources  =  maps_my_ros_datatype_publisher.cpp \
			maps_my_ros_datatype_subscriber.cpp \
			maps_ros_utils.cpp

# Add C sources here
csources = 

PCK      = rtmaps_ros_custom_data_types.pck

# To add options to link:
LDLIBS+=${shell pkg-config --libs roscpp}
LDLIBS+=${shell pkg-config --libs my_data_types}
# LDFLAGS += -L/my/directory/

# To add options to the compiler:
CXXFLAGS += $(shell pkg-config --cflags roscpp)
CXXFLAGS += $(shell pkg-config --cflags my_data_types)
CXXFLAGS += -I../catkin_ws/devel/include
#CFLAGS += ${shell pkg-config --cflags roscpp}
#CFLAGS += ${shell pkg-config --cflags my_data_types}
#CPPFLAGS += -I../catkin_ws/devel/include
# CXXFLAGS += my specific C++ flags

all: pck

RTMAPS_SDKDIR ?=/opt/rtmaps
include $(RTMAPS_SDKDIR)/templates.u/makefile.inc
