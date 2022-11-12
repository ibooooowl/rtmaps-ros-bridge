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
// RTMaps SDK Component
////////////////////////////////

////////////////////////////////
// Purpose of this module :
////////////////////////////////

#include "maps_my_ros_datatype_publisher.h"	// Includes the header of this component

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSmy_ros_datatype_publisher)
    MAPS_INPUT("id",MAPS::FilterInteger32,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSmy_ros_datatype_publisher)
    //MAPS_OUTPUT("oName",MAPS::Integer,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSmy_ros_datatype_publisher)
    MAPS_PROPERTY("topic_name","rtmaps/ros_topic_name",false,false)
    MAPS_PROPERTY_ENUM("published_timestamps","RTMaps samples timestamps|ROS current time",1,false,false)
    MAPS_PROPERTY("frame_id","map",false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSmy_ros_datatype_publisher)
    //MAPS_ACTION("aName",MAPSros_topic_publisher::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (ros_topic_publisher) behaviour
MAPS_COMPONENT_DEFINITION(MAPSmy_ros_datatype_publisher,"my_ros_datatype_publisher","1.0",128,
			  MAPS::Threaded,MAPS::Threaded,
              -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
              -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions



void MAPSmy_ros_datatype_publisher::Birth()
{
	m_first_time = true;
	m_n = nullptr;
	m_pub = nullptr;
	MAPSString topic_name = GetStringProperty("topic_name");
	try 
	{
		m_n = MAPSRosUtils::GetROSNode();
		if (m_n == nullptr)
			Error("Could not create NodeHandle.");
		m_pub = new ros::Publisher();
		if (m_pub == nullptr)
			Error("Could not create Publisher.");
	} 
	catch (ros::Exception const& e ) 
	{
		ReportError(e.what());
	}
		int pub_ts = (int)GetIntegerProperty("published_timestamps");
		if (pub_ts == 0)
			m_publish_rtmaps_timestamp = true;
		else
			m_publish_rtmaps_timestamp = false;

    *m_pub = m_n->advertise<my_data_types::my_data_type>((const char*)topic_name,100);

	m_count = 0;


}

void MAPSmy_ros_datatype_publisher::Core()
{
	MAPSTimestamp t;
    m_ioeltin = StartReading(Input(0));
    if (m_ioeltin == NULL)
        return;
    t = m_ioeltin->Timestamp();

    MAPSString frame_id = (const char*)GetStringProperty("frame_id");
    m_header.frame_id = frame_id.Len() > 0 ? (const char*)frame_id : (const char*)this->Name();
    if (m_publish_rtmaps_timestamp)
        m_header.stamp = MAPSRosUtils::MAPSTimestampToROSTime(t);
    else
        m_header.stamp = ros::Time::now();

    PublishMyMsg();

    m_count++;
}

void MAPSmy_ros_datatype_publisher::PublishMyMsg()
{

    my_data_types::my_data_type msg;
    msg.id = m_ioeltin->Integer32();
    msg.header = m_header;

    m_pub->publish(msg);

}

void MAPSmy_ros_datatype_publisher::Death()
{
	if (m_pub) 
	{
		m_pub->shutdown();
        MAPS_SAFE_DELETE(m_pub);
	}

    if (m_n) 
    {
         m_n = nullptr; //The ROS node will be killed/released by the RTMaps ROS Bridge CoreFunction.
    }

}
