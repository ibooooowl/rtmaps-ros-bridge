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

#include "maps_my_ros_datatype_subscriber.h"	// Includes the header of this component

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSmy_ros_datatype_subscriber)
    //MAPS_INPUT("iName",MAPS::FilterInteger32,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSmy_ros_datatype_subscriber)
    MAPS_OUTPUT("id",MAPS::Integer32,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSmy_ros_datatype_subscriber)
	MAPS_PROPERTY("topic_name","rtmaps/ros_topic_name",false,false)
    MAPS_PROPERTY("transfer_ROS_timestamps",true,false,false)
    MAPS_PROPERTY("subscribe_queue_size",-1,false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSmy_ros_datatype_subscriber)
    //MAPS_ACTION("aName",MAPSmy_data_types_component::ActionName)
MAPS_END_ACTIONS_DEFINITION

// Use the macros to declare this component (my_data_types_component) behaviour
MAPS_COMPONENT_DEFINITION(MAPSmy_ros_datatype_subscriber,"my_ros_datatype_subscriber","1.0",128,
			  MAPS::Threaded,MAPS::Threaded,
			  -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
			  -1, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions

//Initialization: Birth() will be called once at diagram execution startup.			  
void MAPSmy_ros_datatype_subscriber::Birth()
{
    _first_time = true;
    _transfer_ros_timestamp = GetBoolProperty("transfer_ROS_timestamps");

    _n = NULL;
    _sub = NULL;

    _ros = MAPSRosUtils::get();
    if (_ros == NULL)
        Error("Could not init ROS");
    _n =  _ros->get_ros_node();
    if (_n == NULL)
        Error("Could not create ROS node handle.");


    _sub = new ros::Subscriber();
    if (_sub == NULL)
        Error("Could not create ROS Subscriber.");


    int queue_size = (int)GetIntegerProperty("subscribe_queue_size");

    MAPSString topic_name = GetStringProperty("topic_name");

    *_sub = _n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSmy_ros_datatype_subscriber::ROSDataReceivedCallback,this);

	if (_sub == NULL) {
		MAPSStreamedString ss;
        ss << "Could not subscribe to topic " << topic_name;
		Error(ss);
	}

}

void MAPSmy_ros_datatype_subscriber::ROSDataReceivedCallback(const my_data_types::my_data_typeConstPtr& message)
{
    try {
        MAPSTimestamp t = MAPS::CurrentTime();

        MAPSIOElt* ioeltout = StartWriting(Output(0));
        ioeltout->Integer32() = message->id;
        if (_transfer_ros_timestamp) {
            ioeltout->Timestamp() = MAPSRosUtils::ROSTimeToMAPSTimestamp(message->header.stamp);
        } else {
            ioeltout->Timestamp() = t;
        }
        StopWriting(ioeltout);
    } catch (int error) {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}


void MAPSmy_ros_datatype_subscriber::Core()
{
    Wait4Event(isDyingEvent);
}

//De-initialization: Death() will be called once at diagram execution shutdown.
void MAPSmy_ros_datatype_subscriber::Death()
{
    if (_sub) {
        _sub->shutdown();
        MAPS_SAFE_DELETE(_sub);
    }
    if (_n) {
         _ros->release_ros_node();
         _ros->release();
         _n = NULL;
         _ros = NULL;
    }
}
