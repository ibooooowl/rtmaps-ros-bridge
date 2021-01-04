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

#include "maps_ros_utils.h"

std::shared_ptr< MAPSRosUtils* > MAPSRosUtils::s_singleton(nullptr);
MAPSMutex MAPSRosUtils::s_mtx;

std::shared_ptr< MAPSRosUtils* > MAPSRosUtils::get_singleton()
{
    MAPSMutexGuard mtxguard(s_mtx);
    if (s_singleton == nullptr) {
        s_singleton = std::make_shared< MAPSRosUtils* >(new MAPSRosUtils());
        MAPSRosUtils* ru = *s_singleton.get();
        if (false == ru->init_ros()) {
            MAPS::ReportError("Initializing ROS failed.");
            return NULL;
        }
        ru->m_ros_spin_runnable.Init(&MAPSRosUtils::SpinThread, ru);
    }
    return s_singleton;
}

bool MAPSRosUtils::release()
{
    MAPSMutexGuard mtxguard(s_mtx);
    if (s_singleton == nullptr || s_singleton.use_count() == 0) {
        return false;
    }
    if (s_singleton.use_count() == 0) {
        if ((*s_singleton)->m_rosnode_refcount > 0) {
            MAPS::ReportError("The MAPSRosUtils singleton should not be released while references to the ROS node are still valid.");
            return false;
        }
        delete s_singleton.get();
        s_singleton = nullptr;
    }
    return true;
}

bool MAPSRosUtils::init_ros()
{
    try {
        ros::VP_string remappings;
        MAPSProperty* pid_prop = MAPS::Property("Engine.pid");
        int current_pid = 0;
        if (pid_prop != NULL)
            current_pid = MAPS::Property("Engine.pid")->IntegerValue();
        MAPSStreamedString ss;
        ss << "rtmaps_ros_bridge_" << current_pid;
        ros::init(remappings,(const char*)ss,ros::init_options::NoSigintHandler);
        MAPS::ReportInfo("ROS correctly initialized."); 
    } catch (ros::Exception& e) {
        MAPS::ReportError(e.what());
        return false;
    }
    return true;
}

ros::NodeHandle* MAPSRosUtils::get_ros_node()
{
    MAPSMutexGuard mtxguard(s_mtx);
    if (m_node == NULL) {
        m_node = new ros::NodeHandle();
        m_ros_spin_runnable.Start();
    }
    return m_node;
}

bool MAPSRosUtils::release_ros_node()
{
    MAPSMutexGuard mtxguard(s_mtx);
    if (m_node == NULL) {
        return false;
    }
    if (s_singleton.use_count() == 0) {
        ros::shutdown();
        m_ros_spin_runnable.Stop();
        delete m_node;
        m_node = nullptr;
    }
    return true;
}

void* MAPSRosUtils::SpinThread(void* instancePtr)
{
    ros::spin();

    return this; // for skipping a warning at compilation
}
