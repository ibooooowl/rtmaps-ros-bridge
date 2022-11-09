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
	MAPSros_clock(const char *componentName, MAPSComponentDefinition& md):MAPSComponent(componentName,md), MAPSBaseClock(componentName) {m_clock_disabled=false; m_clock_has_shutdown=true;}
	virtual ~MAPSros_clock() {}
	void Set(MAPSProperty& p, MAPSInt64 value);
private :
	// Place here your specific methods and attributes
	MAPSROSBridgeCoreFunctionInterface* m_ros_bridge_cf;
	ros::NodeHandle*  	m_ros_node;
	bool 				m_clock_disabled;
	bool 				m_clock_has_shutdown;
	MAPSMutex 			m_time_monitor;

	void InitClock();
	void RunClock();
	void ShutdownClock();
	MAPSTimestamp CurrentTime();

};

#endif
