#ifndef MAPS_ROS_BRIDGE_CORE_FUNCTION_INTERFACE_H
#define MAPS_ROS_BRIDGE_CORE_FUNCTION_INTERFACE_H
#include "maps.hpp"

#include "ros/ros.h"


static inline ros::Time MAPSTimestampToROSTime(MAPSTimestamp t) {ros::Time rostime; rostime.sec = t/1000000; rostime.nsec = (t%1000000)*1000; return rostime;}
static inline MAPSTimestamp ROSTimeToMAPSTimestamp(ros::Time t) {MAPSTimestamp ts; ts = ((MAPSInt64)t.sec)*1000000 + t.nsec/1000; return ts;}


class MAPSROSBridgeCoreFunctionInterface
{
protected:
	MAPSROSBridgeCoreFunctionInterface() {}
	MAPSROSBridgeCoreFunctionInterface(const MAPSROSBridgeCoreFunctionInterface&) {}
	virtual ~MAPSROSBridgeCoreFunctionInterface() {}
public:
    virtual ros::NodeHandle* GetROSNode() = 0;
   
};

#endif