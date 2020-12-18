#include "maps_ros_utils.h"

MAPSRosUtils* MAPSRosUtils::s_singleton = NULL;
MAPSMutex MAPSRosUtils::s_mtx;

MAPSRosUtils* MAPSRosUtils::get()
{
    MAPSMutexGuard mtxguard(s_mtx);
    if (s_singleton == NULL) {
        s_singleton = new MAPSRosUtils();
        if (false == s_singleton->init_ros()) {
            MAPS::ReportError("Initializing ROS failed.");
            return NULL;
        }
        s_singleton->m_ros_spin_runnable.Init(&MAPSRosUtils::SpinThread,s_singleton);
    }
    s_singleton->m_singleton_refcount++;
    return s_singleton;
}

bool MAPSRosUtils::release()
{
    MAPSMutexGuard mtxguard(s_mtx);
    if (s_singleton == NULL || s_singleton->m_singleton_refcount < 1) {
        return false;
    }
    s_singleton->m_singleton_refcount--;
    if (s_singleton->m_singleton_refcount == 0) {
        if (s_singleton->m_rosnode_refcount > 0) {
            MAPS::ReportError("The MAPSRosUtils singleton should not be released while references to the ROS node are still valid.");
            s_singleton->m_singleton_refcount++;
            return false;
        }
        delete s_singleton;
        s_singleton = NULL;
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
    } catch (ros::Exception e) {
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
    m_rosnode_refcount++;
    return m_node;
}

bool MAPSRosUtils::release_ros_node()
{
    MAPSMutexGuard mtxguard(s_mtx);
    if (m_node == NULL || m_rosnode_refcount < 1) {
        return false;
    }
    m_rosnode_refcount--;
    if (m_rosnode_refcount == 0) {
        ros::shutdown();
        m_ros_spin_runnable.Stop();
        delete m_node;
        m_node = NULL;
    }
    return true;
}

void* MAPSRosUtils::SpinThread(void* instancePtr)
{
    ros::spin();
}
