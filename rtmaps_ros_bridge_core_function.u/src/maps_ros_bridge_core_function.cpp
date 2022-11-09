#include "maps_ros_bridge_core_function.h"


MAPS_CF_DEFINITION(MAPSROSBridgeCoreFunction,MAPSCoreFunction,ros_bridge_core_function,"1.0.0")

MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSROSBridgeCoreFunction)
MAPS_END_PROPERTIES_DEFINITION

MAPS_BEGIN_ACTIONS_DEFINITION(MAPSROSBridgeCoreFunction)
MAPS_END_ACTIONS_DEFINITION

MAPSCoreFunction* MAPSROSBridgeCoreFunction::InstFunct(const char *name,MAPSCFDefinition* kf)
{
    if (kf->refCounter==0) {
        ++kf->refCounter;
        return new MAPSROSBridgeCoreFunction(name);
    }
    else {
        MAPS::ReportError("The ROS bridge kernel function has already been loaded. Cannot load another one.",0);
        return NULL;
    }
}

MAPSROSBridgeCoreFunction::MAPSROSBridgeCoreFunction(const char* name)
: MAPSCoreFunction(name)
{
    m_node = nullptr;
    m_ros_spin_runnable.Init(&MAPSROSBridgeCoreFunction::SpinThread, this);
}

ros::NodeHandle* MAPSROSBridgeCoreFunction::GetROSNode()
{
    MAPSMutexGuard mtx(m_ros_mtx);
    if (m_node != nullptr)
    {
        return m_node;
    }
    m_evt_ros_node_started.Reset();
    m_ros_spin_runnable.Start();
    m_evt_ros_node_started.Wait();
    return m_node;
}

/*****************************************************************************
** MAPSCoreFunction implementation
*****************************************************************************/

void* MAPSROSBridgeCoreFunction::GetInterface()
{
	return static_cast<MAPSROSBridgeCoreFunctionInterface*>(this);
}


void MAPSROSBridgeCoreFunction::CallbackRun()
{
    GetROSNode();
}

void MAPSROSBridgeCoreFunction::CallbackShutdown()
{
    ros::shutdown();
    m_ros_spin_runnable.Stop();
}


void* MAPSROSBridgeCoreFunction::SpinThread(void* )
{
    try {
        ros::VP_string remappings;
        MAPSProperty* pid_prop = MAPS::Property("Engine.pid");
        int current_pid = 0;
        if (pid_prop != nullptr)
            current_pid = MAPS::Property("Engine.pid")->IntegerValue();
        MAPSStreamedString ss;
        ss << "rtmaps_ros_bridge_" << current_pid;

        ros::init(remappings,(const char*)ss,ros::init_options::NoSigintHandler);

        if (m_node == nullptr) 
        {
            if (ros::master::check() == false)
            {
                ReportError("ROS Master doesn't seem to be running. Please start roscore before running the RTMaps diagram.")    ;
                m_evt_ros_node_started.Set();
                return nullptr;
             }
            else
            {
                m_node = new ros::NodeHandle();
            }
        }
        m_evt_ros_node_started.Set();
    }
    catch (ros::Exception& e) 
    {
        ReportError(e.what());
        return nullptr;
    }

    //Loop until shutdown.
    ros::spin();
    MAPS_SAFE_DELETE(m_node);
    return this; // for skipping a warning at compilation
}
