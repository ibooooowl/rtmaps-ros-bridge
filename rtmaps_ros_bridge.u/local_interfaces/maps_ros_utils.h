/*
 * Copyright (c) 2020 Intempora
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    * Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of Intempora nor the names of its contributors may be
 *      used to endorse or promote products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL INTEMPORA
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __MAPS_ROS_UTILS_H__
#define __MAPS_ROS_UTILS_H__

#include "maps.hpp"
#include "maps_ros_defines.h"



class MAPSRosUtils {
    static std::shared_ptr< MAPSRosUtils* > s_singleton;
    static MAPSMutex    s_mtx;
    int                 m_rosnode_refcount;

    ros::NodeHandle*         m_node; //One node per process.

    MAPSRunnable<MAPSRosUtils> m_ros_spin_runnable;


    bool init_ros();
    void* SpinThread(void* instancePtr);

public:
    MAPSRosUtils() : m_rosnode_refcount(0), m_node(NULL) {}

    static std::shared_ptr< MAPSRosUtils* > get_singleton();
    static bool release();


    ros::NodeHandle* get_ros_node();
    bool release_ros_node();

    static inline ros::Time MAPSTimestampToROSTime(MAPSTimestamp t) {ros::Time rostime; rostime.sec = t/1000000; rostime.nsec = (t%1000000)*1000; return rostime;}
    static inline MAPSTimestamp ROSTimeToMAPSTimestamp(ros::Time t) {MAPSTimestamp ts; ts = ((MAPSInt64)t.sec)*1000000 + t.nsec/1000; return ts;}

};




#endif //__MAPS_ROS_UTILS_H__
