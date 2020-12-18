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
