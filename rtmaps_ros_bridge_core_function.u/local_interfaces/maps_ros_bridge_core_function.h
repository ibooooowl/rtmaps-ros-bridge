#ifndef __MAPS_ROS_BRIDGE_CORE_FUNCTION_H__
#define __MAPS_ROS_BRIDGE_CORE_FUNCTION_H__

#include "maps.h"
#include "maps_corefunction.h"
#include "maps_runshutdownlistener.h"

#include "ros/ros.h"

#include "maps_ros_bridge_core_function_interface.h"

class MAPSROSBridgeCoreFunction : public MAPSCoreFunction, 
			 public MAPSRunShutdownListener, public MAPSROSBridgeCoreFunctionInterface
{
	MAPS_CF_STANDARD_HEADER_CODE()

public:
    MAPSROSBridgeCoreFunction(const char *name);
    virtual ~MAPSROSBridgeCoreFunction() {}

	//MAPSRunShutdownListener interface
	void CallbackRun();
	void CallbackShutdown();

public: // MAPSCoreFunction
	virtual void* GetInterface();

protected :
    MAPSMutex               m_ros_mtx;
    MAPSEvent               m_evt_ros_node_started;
    ros::NodeHandle*        m_node; //One node per process.

    MAPSRunnable<MAPSROSBridgeCoreFunction> m_ros_spin_runnable;
    void* SpinThread(void* instancePtr);

    //Core Function interface
    ros::NodeHandle* GetROSNode();


};

#endif
