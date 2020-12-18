////////////////////////////////
// RTMaps SDK Component header
////////////////////////////////

#ifndef _Maps_ros_clock_H
#define _Maps_ros_clock_H

// Includes maps sdk library header
#include "maps.hpp"
#include "maps_baseclock.h"
#include "maps_ros_defines.h"
#include "maps_ros_utils.h"

// Declares a new MAPSComponent child class
class MAPSros_clock : public MAPSComponent, public MAPSBaseClock
{
	// Use standard header definition macro
	MAPS_COMPONENT_HEADER_CODE_WITHOUT_CONSTRUCTOR(MAPSros_clock)
	MAPSros_clock(const char *componentName, MAPSComponentDefinition& md):MAPSComponent(componentName,md), MAPSBaseClock(componentName) {_clockDisabled=false; _clockHasShutdown=true;}
	void Set(MAPSProperty& p, MAPSInt64 value);
private :
	// Place here your specific methods and attributes
    MAPSRosUtils* _ros;
	bool _clockDisabled;
	bool _clockHasShutdown;
	MAPSMutex m_MyTimeMonitor;

	void InitClock();
	void RunClock();
	void ShutdownClock();
	MAPSTimestamp CurrentTime();

};

#endif
