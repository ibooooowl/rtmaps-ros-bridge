# RTMaps Makefile
# Copyright 2002-2021 (C) Intempora S.A.

ROOTDIR  := $(realpath $(CURDIR)/..)
ifeq ($(ROOTDIR),)
$(error Please use make at least version 3.81)
endif

# Add C++ sources here
sources  =  maps_ros_topic_subscriber.cpp maps_ros_topic_publisher.cpp maps_ros_array_layout_infos_splitter.cpp maps_ros_utils.cpp maps_ros_clock.cpp

# Add C sources here
csources = 

PCK      =  rtmaps_ros_bridge.pck

# To add options to link:
LDLIBS+= ${shell pkg-config --libs roscpp}
LDLIBS+= ${shell pkg-config --libs sensor_msgs}
LDLIBS+= ${shell pkg-config --libs geometry_msgs}
LDLIBS+= ${shell pkg-config --libs nav_msgs}

ifdef CROSS_COMPILE 
  $(info CROSS_COMPILE is ON)
ifndef ROS_VERSION 
  $(error ROS_VERSION is undefined)
endif
  override LDFLAGS += -Wl,-rpath=/opt/ros/${ROS_VERSION}/lib
endif


#LDLIBS+=-L/opt/ros/lunar/lib -lroscpp -lpthread -lboost_signals -lboost_filesystem -lboost_system -lrosconsole -lrosconsole_log4cxx -lrosconsole_backend_interface -llog4cxx -lboost_regex -lboost_system -lboost_thread -lboost_chrono -lboost_date_time -lboost_atomic -lpthread -lroscpp_serialization -lrostime -lboost_date_time -lboost_system -lboost_thread -lboost_chrono -lboost_atomic -lpthread -lxmlrpcpp -lcpp_common -lboost_system -lboost_thread -lboost_chrono -lboost_date_time -lboost_atomic -lpthread -lconsole_bridge
#LDLIBS+=-L/opt/ros/lunar/lib -lroscpp_serialization -lrostime -lboost_date_time -lboost_system -lboost_thread -lboost_chrono -lboost_atomic -lpthread -lcpp_common -lboost_system -lboost_thread -lboost_chrono -lboost_date_time -lboost_atomic -lpthread -lconsole_bridge
#LDLIBS+=-L/opt/ros/lunar/lib -lroscpp_serialization -lrostime -lboost_date_time -lboost_system -lboost_thread -lboost_chrono -lboost_atomic -lpthread -lcpp_common -lboost_system -lboost_thread -lboost_chrono -lboost_date_time -lboost_atomic -lpthread -lconsole_bridge

# LDFLAGS += -L/my/directory/

# To add options to the compiler:
# CFLAGS   += my specific C flags
# CXXFLAGS += my specific C++ flags

CXXFLAGS += $(shell pkg-config --cflags roscpp)
CXXFLAGS += $(shell pkg-config --cflags sensor_msgs)
CXXFLAGS += $(shell pkg-config --cflags geometry_msgs)
CXXFLAGS += $(shell pkg-config --cflags nav_msgs)

all: pck

RTMAPS_SDKDIR ?=/opt/rtmaps
include $(RTMAPS_SDKDIR)/templates.u/makefile.inc
