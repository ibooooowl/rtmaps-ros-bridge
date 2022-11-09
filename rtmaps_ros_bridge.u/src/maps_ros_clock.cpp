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


#include "maps_ros_clock.h"	// Includes the header of this component
#include "maps_corefunction.h"


// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSros_clock)
    //MAPS_INPUT("iName",MAPS::FilterInteger,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSros_clock)
    MAPS_OUTPUT("ros_time",MAPS::Integer64,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSros_clock)
    MAPS_PROPERTY("max_timespeed",100,false,true)
	MAPS_PROPERTY_SUBTYPE("ros_time_publication_period",-1,false,true,MAPS::PropertySubTypeTime)
	MAPS_PROPERTY_SUBTYPE("ros_use_sim_time_init_timeout",10000000,false,false,MAPS::PropertySubTypeTime)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSros_clock)
    //MAPS_ACTION("aName",MAPSros_clock::ActionName)
MAPS_END_ACTIONS_DEFINITION

//Version 1.0.1: corrected ROS time to RTMaps time conversion.
//Version 1.1.0: support for the  ROS simulation time

// Use the macros to declare this component (ros_clock) behaviour
MAPS_COMPONENT_DEFINITION(MAPSros_clock,"ros_clock","1.1.0",128,
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
}

void MAPSros_clock::RunClock()
{
	m_ros_node = MAPSRosUtils::GetROSNode();	
	if (m_ros_node == nullptr)
	{
		m_clock_disabled = true;
		ReportError("Could not init ROS.");
		return;
	}

	m_clock_has_shutdown=false;

    SetAbsoluteTimeSpeed((int)GetIntegerProperty("max_timespeed")*10);
	if (MAPS::GetFirstTimestamp() != 0 || MAPS::GetLastTimestamp() != 0) //We've got a Player in the diagram...
	{
		m_clock_disabled = true;
		ReportError("Trying to synchronize the RTMaps clock with the GPS UTC time is not allowed in replay mode.");
	}

    ros::Time::init();

	bool use_sim_time;
	bool res = ros::param::get("use_sim_time",use_sim_time);
	if (res == true)
	{
		MAPSTimestamp ros_time = 0;
		MAPSTimestamp current_time = 0;

		bool is_ok = false;
		int n_tries = GetIntegerProperty("ros_use_sim_time_init_timeout") / 100000;
		ReportInfo("Initializing clock against ROS simulation time. Waiting for ROS simulation time validity...");
		for (int count = 0; count < n_tries; count++)
		{
			ros_time = ROSTimeToMAPSTimestamp(ros::Time::now());
			MAPSAbsoluteTime ros_time_at;
			MAPS::Timestamp2AbsoluteTimeUTC(ros_time, &ros_time_at);
			MAPSAbsoluteTime at;
			MAPS::GetAbsoluteTimeUTC(&at);
			current_time = MAPS::AbsoluteTimeUTC2Timestamp(&at);

			if (abs(ros_time - current_time) > 2000000)
			{
				is_ok = true;
				break;
			}
			MAPS::Sleep(100000);
		}

		if (false == is_ok)
		{
			ReportError("It seems we should be using the ROS simulation time (ROS param use_sim_time is true)\n\
			but ROS keeps on reporting the wall time. Make sure the rosbag play is started with argument --clock.\n\
			Otherswise it would prevent the RTMaps recorder from working in case\n\
			we get ROS data samples with a timestamp that is far from the wall time (e.g. if their timestamps\n\
			correspond to the simulation time), and also if later the ROS time jumps backwards to simulation time.");

			MAPS::AsynchParse("shutdown");
		} 
		else 
		{
			ReportInfo("ROS simulation time initialization ok.");
		}
	}
}

MAPSTimestamp MAPSros_clock::CurrentTime()
{
	if (true == m_clock_disabled || true == m_clock_has_shutdown)
		return 0;
	m_time_monitor.Lock();
    MAPSTimestamp t = ROSTimeToMAPSTimestamp(ros::Time::now());
	m_time_monitor.Release();
	return t;
}

void MAPSros_clock::Birth()
{
	if (m_clock_disabled == true || m_clock_has_shutdown == true)
	{
    	ros::Time::init(); // the ros::time has not been initialized yet (ROS clock is not active in RTMaps). We need to call it once would it be only to publish it on the output.
	}
}

void MAPSros_clock::Core() 
{
	MAPSInt64 t = GetIntegerProperty("ros_time_publication_period");
	if(t!=-1)
		Rest(t);
	else
		Wait4Event(isDyingEvent);

	MAPSIOElt* ioeltout = StartWriting(Output(0));
	ioeltout->Integer64() = ROSTimeToMAPSTimestamp(ros::Time::now());
	StopWriting(ioeltout);
}

void MAPSros_clock::Death()
{
}

void MAPSros_clock::ShutdownClock()
{
	m_ros_node = nullptr;
	m_ros_bridge_cf = nullptr;

    m_clock_has_shutdown = true;
	m_clock_disabled = false;
 }
