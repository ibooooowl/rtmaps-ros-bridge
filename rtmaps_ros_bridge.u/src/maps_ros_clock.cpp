////////////////////////////////
// RTMaps SDK Component
////////////////////////////////

////////////////////////////////
// Purpose of this module :
////////////////////////////////

#include "maps_ros_clock.h"	// Includes the header of this component

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSros_clock)
    //MAPS_INPUT("iName",MAPS::FilterInteger,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSros_clock)
    //MAPS_OUTPUT("oName",MAPS::Integer,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSros_clock)
    MAPS_PROPERTY("max_timespeed",100,false,true)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSros_clock)
    //MAPS_ACTION("aName",MAPSros_clock::ActionName)
MAPS_END_ACTIONS_DEFINITION

//Version 1.0.1: corrected ROS time to RTMaps time conversion.

// Use the macros to declare this component (ros_clock) behaviour
MAPS_COMPONENT_DEFINITION(MAPSros_clock,"ros_clock","1.0.1",128,
			  MAPS::Threaded,MAPS::Threaded,
			  -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
			  -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions


void MAPSros_clock::Set(MAPSProperty& p, MAPSInt64 value)
{
	if (&p == &Property("max_timespeed")) {

	}
	MAPSComponent::Set(p,value);
}

void MAPSros_clock::InitClock()
{
 /*   if (false == g_ros_utils.init_ros()) {
		_clockDisabled = true;
		ReportError("Could not initialize ROS.");
    }*/
}

void MAPSros_clock::RunClock()
{
	_clockHasShutdown=false;

    SetAbsoluteTimeSpeed((int)GetIntegerProperty("max_timespeed")*10);
	if (MAPS::GetFirstTimestamp() != 0 || MAPS::GetLastTimestamp() != 0) //We've got a Player in the diagram...
	{
		_clockDisabled = true;
		ReportError("Trying to synchronize the RTMaps clock with the GPS UTC time is not allowed in replay mode.");
	}
    _ros = MAPSRosUtils::get();
    if (NULL == _ros) {
		_clockDisabled = true;
		ReportError("Could not initialize ROS.");
	}
    ros::Time::init();
}

MAPSTimestamp MAPSros_clock::CurrentTime()
{
	if (true == _clockHasShutdown)
		return 0;
	m_MyTimeMonitor.Lock();
    MAPSTimestamp t = MAPSRosUtils::ROSTimeToMAPSTimestamp(ros::Time::now());
	m_MyTimeMonitor.Release();
	return t;
}

void MAPSros_clock::Birth()
{
}

void MAPSros_clock::Core() 
{
	Wait4Event(isDyingEvent);
}

void MAPSros_clock::Death()
{
}

void MAPSros_clock::ShutdownClock()
{
    _ros->release();
    _clockHasShutdown = true;
	_clockDisabled = false;
}
