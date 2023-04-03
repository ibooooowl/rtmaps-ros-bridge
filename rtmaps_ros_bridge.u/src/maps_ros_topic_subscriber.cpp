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

#include "maps_ros_topic_subscriber.h"	// Includes the header of this component

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSros_topic_subscriber)
    //MAPS_INPUT("iName",MAPS::FilterInteger,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSros_topic_subscriber)
    MAPS_OUTPUT("output_text",MAPS::TextAscii,NULL,NULL,0)
    MAPS_OUTPUT_FIFOSIZE("output_image",MAPS::IplImage,NULL,NULL,0,10)
    MAPS_OUTPUT_FIFOSIZE("output_compressed_image",MAPS::MAPSImage,NULL,NULL,0,10)
    MAPS_OUTPUT("output_int32",MAPS::Integer32,NULL,NULL,1)
    MAPS_OUTPUT("output_int32_array",MAPS::Integer32,NULL,NULL,0)
    MAPS_OUTPUT("output_int64",MAPS::Integer64,NULL,NULL,1)
    MAPS_OUTPUT("output_int64_array",MAPS::Integer64,NULL,NULL,0)
    MAPS_OUTPUT("output_float32",MAPS::Float32,NULL,NULL,1)
    MAPS_OUTPUT("output_float32_array",MAPS::Float32,NULL,NULL,0)
    MAPS_OUTPUT("output_float64",MAPS::Float64,NULL,NULL,1)
    MAPS_OUTPUT("output_float64_array",MAPS::Float64,NULL,NULL,0)
	MAPS_OUTPUT_USER_STRUCTURE("array_layout",ROSArrayLayout)
    //CAN
    MAPS_OUTPUT("can_frames",MAPS::CANFrame,NULL,NULL,1)
    //LASER SCAN
	MAPS_OUTPUT("output_laser_scan_ranges",MAPS::Float64,NULL,NULL,0)
	MAPS_OUTPUT("output_laser_scan_intensities",MAPS::Float64,NULL,NULL,0)
	MAPS_OUTPUT("output_laser_scan_info",MAPS::Float64,NULL,NULL,7)
	//POINT CLOUD
	MAPS_OUTPUT("output_point_cloud",MAPS::Float32,NULL,NULL,0)
	MAPS_OUTPUT("output_point_cloud_channel_sizes",MAPS::Integer32,NULL,NULL,0)
	MAPS_OUTPUT("output_point_cloud_channels",MAPS::Float32,NULL,NULL,0)
	//POINT CLOUD 2
	MAPS_OUTPUT("output_point_cloud_2_info",MAPS::Integer32,NULL,NULL,6) //width, height, is_big_endian, point_step, row_step, is_dense
	MAPS_OUTPUT("output_point_cloud_2_fields_names", MAPS::TextAscii,NULL,NULL,0) // fields names, separated by '|'
	MAPS_OUTPUT("output_point_cloud_2_fields_info", MAPS::Integer32,NULL,NULL,0) // vector of (offset, datatype, count)
	MAPS_OUTPUT("output_point_cloud_2_data",MAPS::Stream8,NULL,NULL,0) //data
	//JOYSTICK
	MAPS_OUTPUT("output_joy_axes",MAPS::Float32,NULL,NULL,0)
	MAPS_OUTPUT("output_joy_buttons",MAPS::Integer32,NULL,NULL,0)
	//TWIST
	MAPS_OUTPUT("output_twist",MAPS::Float64,NULL,NULL,6)
    //COVARIANCE
    MAPS_OUTPUT("output_covariance",MAPS::Float64,NULL,NULL,36)
	//IMU MESSAGES
	MAPS_OUTPUT("orientation_quaternion",MAPS::Float64,NULL,NULL,4)
	MAPS_OUTPUT("angular_velocities",MAPS::Float64,NULL,NULL,3)
	MAPS_OUTPUT("accelerations",MAPS::Float64,NULL,NULL,3)
	//RANGE MESSAGES
	MAPS_OUTPUT("range_info",MAPS::Float64,NULL,NULL,4)
    //NAVSATFIX MESSAGES
    MAPS_OUTPUT("navsatfix_status",MAPS::Integer32,NULL,NULL,2)
    MAPS_OUTPUT("navsatfix_lla_pos",MAPS::Float64,NULL,NULL,3)
    MAPS_OUTPUT("navsatfix_pos_cov",MAPS::Float64,NULL,NULL,9)
    MAPS_OUTPUT("navsatfix_pos_cov_type",MAPS::Integer32,NULL,NULL,1)
    //VISUALIZATION MARKERS
    MAPS_OUTPUT("output_marker_arrow", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_cube", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_sphere", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_cylinder", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_line_strip", MAPS::Float64, NULL,NULL, 0)
    MAPS_OUTPUT("output_marker_line_list", MAPS::Float64,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_cube_list", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_sphere_list", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_points", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_text_view_facing", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_mesh_resource", MAPS::RealObject,NULL,NULL,0)
    MAPS_OUTPUT("output_marker_triangle_list", MAPS::Float64,NULL,NULL,0)

    // RMP Battery Messages
    MAPS_OUTPUT("output_battery_charge_state",MAPS::Float32,NULL,NULL,5)
    MAPS_OUTPUT("output_battery_temperature",MAPS::Float32,NULL,NULL,5)
    MAPS_OUTPUT("output_battery_min_propulsion_charge_state",MAPS::Float32,NULL,NULL,1)
    MAPS_OUTPUT("output_battery_aux_battery_voltage",MAPS::Float32,NULL,NULL,1)
    MAPS_OUTPUT("output_battery_aux_battery_current",MAPS::Float32,NULL,NULL,1)
    MAPS_OUTPUT("output_battery_abb_system_status",MAPS::UnsignedInteger32,NULL,NULL,1)
    MAPS_OUTPUT("output_battery_aux_battery_status",MAPS::UnsignedInteger32,NULL,NULL,1)

    // RMP Motor Status Messages
    MAPS_OUTPUT("output_motor_status_current",MAPS::Float32,NULL,NULL,4)
    MAPS_OUTPUT("output_motor_status_current_limit",MAPS::Float32,NULL,NULL,4)
    MAPS_OUTPUT("output_motor_status_mcu_inst_power",MAPS::Float32,NULL,NULL,4)
    MAPS_OUTPUT("output_motor_status_mcu_total_energy",MAPS::Float32,NULL,NULL,4)
    MAPS_OUTPUT("output_motor_status_max_current",MAPS::Float32,NULL,NULL,1)
    MAPS_OUTPUT("output_motor_status_min_current_limit",MAPS::Float32,NULL,NULL,1)

    // RMP Fault Status Messages
    MAPS_OUTPUT("output_fault_status",MAPS::TextAscii,NULL,NULL,0)

    // RMP Audio Command Messages
    MAPS_OUTPUT("output_audio_command_command",MAPS::UnsignedInteger32,NULL,NULL,1)

    // RMP Bool Messages
    MAPS_OUTPUT("output_bool",MAPS::UnsignedInteger8,NULL,NULL,1)


MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSros_topic_subscriber)
	MAPS_PROPERTY("topic_name","rtmaps/ros_topic_name",false,false)
	MAPS_PROPERTY_ENUM("topic_type","None",0,false,false)
	MAPS_PROPERTY_ENUM("message","None",0,false,false) //Text|Image|Int 32|Int 32 array|Int 64|Int 64 array|Float 32|Float 32 array|Float 64|Float 64 array|Laser scan|Twist",0,false,false)
    MAPS_PROPERTY("subscribe_queue_size",-1,false,false)
    MAPS_PROPERTY("transfer_ROS_timestamps",true,false,false)
    MAPS_PROPERTY("max_text_length",255,false,false)
    MAPS_PROPERTY("max_array_size",256,false,false)
    MAPS_PROPERTY("laser_discard_out_or_range_data",false,false,false)
    MAPS_PROPERTY("max_nb_points",-1,false,false)
    MAPS_PROPERTY("compressed_image_width",640,false,false)
    MAPS_PROPERTY("compressed_image_height",480,false,false)
MAPS_END_PROPERTIES_DEFINITION



// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSros_topic_subscriber)
    //MAPS_ACTION("aName",MAPSros_topic_subscriber::ActionName)
MAPS_END_ACTIONS_DEFINITION

//Version 1.1 : corrected potential bug with non-aligned images.
//Version 1.2 : fixed encoding of images "rgba8" and "bgra8" instead of "rgba" and "bgra"
//Version 1.3 : rewritten way to specify topic type and message type.
//Version 1.4 : added IMU output support.
//Version 1.5 : added PointCloud
//Version 1.6 : added Range
//Version 1.7 : added Pose and PoseStamped
//Version 1.8 : added PointCloud2
//Version 1.9 : added Odometry support (from nav_msgs)
//Version 2.0 : added PointCloud2 output in the form of XYZ points.
//Version 2.1 : added max_nb_points property for PointCloud2
//Version 2.2 : supports PointCloud2 with padding or something (number of fields * sizeof(data_type) does not match point_step)
//Version 2.3 : added NavSatFix support (from sensor_msgs)
//Version 2.4 : changed way ROS nodes are handled (using singleton), no more intercepting Ctrl+C, fixed potential crash at shutdown (try catch in callbacks from ROS node thread).
//Version 2.5 : added support for TwistStamped msg
//Version 2.5.1: corrected ros time to RTMaps time conversion
// Use the macros to declare this component (ros_topic_subscriber) behaviour
MAPS_COMPONENT_DEFINITION(MAPSros_topic_subscriber,"ros_topic_subscriber","2.5.2",128,
			  MAPS::Threaded,MAPS::Threaded,
			  -1, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  0, // Nb of outputs. Leave -1 to use the number of declared output definitions
              4, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions


MAPSros_topic_subscriber::MAPSros_topic_subscriber(const char* name, MAPSComponentDefinition& cd) :
MAPSComponent(name,cd)
{
	MAPSEnumStruct topic_types;
    for (unsigned int i=0; i< sizeof(s_topic_types)/sizeof(const char*); i++) 
    {
		topic_types.enumValues->Append() = s_topic_types[i];
	}
	DirectSet(Property("topic_type"),topic_types);
}

void MAPSros_topic_subscriber::Dynamic()
{
	m_topic_type = (int)GetIntegerProperty("topic_type");
	int selected_message = (int)GetIntegerProperty("message");
    bool use_default_message_idx = false;
	if (Property("topic_type").PropertyChanged()) 
    {
		Property("topic_type").AcknowledgePropertyChanged();
        use_default_message_idx = true;
	}
	MAPSEnumStruct messages;

	switch (m_topic_type) 
    {
	case TOPIC_TYPE_STD:
        for (unsigned int i=0; i < sizeof(s_std_msgs)/sizeof(const char*); i++) 
        {
			messages.enumValues->Append() = s_std_msgs[i];
		}
        if (use_default_message_idx)
            selected_message = 0;
		break;
	case TOPIC_TYPE_SENSOR:
        for (unsigned int i=0; i < sizeof(s_sensor_msgs)/sizeof(const char*); i++) 
        {
			messages.enumValues->Append() = s_sensor_msgs[i];
		}
        if (use_default_message_idx)
            selected_message = SENSOR_MSG_POINT_CLOUD2;
        break;
	case TOPIC_TYPE_GEOM:
        for (unsigned int i=0; i < sizeof(s_geometry_msgs)/sizeof(const char*); i++) 
        {
			messages.enumValues->Append() = s_geometry_msgs[i];
		}
        if (use_default_message_idx)
            selected_message = 0;
        break;
    case TOPIC_TYPE_NAV:
        for (unsigned int i=0; i < sizeof(s_nav_msgs)/sizeof(const char*); i++) 
        {
            messages.enumValues->Append() = s_nav_msgs[i];
        }
        if (use_default_message_idx)
            selected_message = 0;
        break;
    case TOPIC_TYPE_VISU:
        for (unsigned int i=0; i < sizeof(s_visu_msgs)/sizeof(const char*); i++) 
        {
            messages.enumValues->Append() = s_visu_msgs[i];
        }
        if (use_default_message_idx)
            selected_message = 0;
        break;
    case TOPIC_TYPE_CAN:
        for (unsigned int i=0; i < sizeof(s_can_msgs)/sizeof(const char*); i++) 
        {
            messages.enumValues->Append() = s_can_msgs[i];
        }
        if (use_default_message_idx)
            selected_message = 0;
        break;
    case TOPIC_TYPE_RMP:
        for (unsigned int i=0; i < 3; i++)
        {
            messages.enumValues->Append() = s_rmp_msgs[i];
        }
        if (use_default_message_idx)
            selected_message = 0;
        break;
    default :
		messages.enumValues->Append() = "None";
		ReportError("This topic type is not supported yet.");
		break;
	}
	if (selected_message >= messages.enumValues->Size())
		selected_message = 0;
	messages.selectedEnum = selected_message;
	DirectSet(Property("message"),messages);
	m_message = selected_message;


	m_ros_header_avail = false;

	switch(m_topic_type) 
    {
	case TOPIC_TYPE_STD:
		CreateIOsForStdTopics(&m_ros_header_avail);
		break;
	case TOPIC_TYPE_SENSOR:
		CreateIOsForSensorTopics(&m_ros_header_avail);
		break;
	case TOPIC_TYPE_GEOM:
		CreateIOsForGeomTopics(&m_ros_header_avail);
        break;
    case TOPIC_TYPE_NAV:
        CreateIOsForNavTopics(&m_ros_header_avail);
        break;
    case TOPIC_TYPE_VISU:
        CreateIOsForVisuTopics(&m_ros_header_avail);
        break;
    case TOPIC_TYPE_CAN:
        CreateIOsForCANTopics(&m_ros_header_avail);
        break;
    case TOPIC_TYPE_RMP:
        CreateIOsForRmpTopics(&m_ros_header_avail);
        break;
    default:
		ReportError("Topic is not supported yet.");
	}
	if (m_ros_header_avail)
		NewProperty("transfer_ROS_timestamps");
}

void MAPSros_topic_subscriber::CreateIOsForStdTopics(bool* ros_header_avail)
{
	*ros_header_avail = false;
	switch(m_message) 
    {
	case STD_MSG_TEXT :
		NewOutput("output_text");
		NewProperty("max_text_length");
		break;
	case STD_MSG_INT32 :
		NewOutput("output_int32");
		break;
	case STD_MSG_INT32_ARRAY :
		NewOutput("output_int32_array");
		NewOutput("array_layout");
		NewProperty("max_array_size");
		break;
	case STD_MSG_INT64 :
		NewOutput("output_int64");
		break;
	case STD_MSG_INT64_ARRAY :
		NewOutput("output_int64_array");
		NewOutput("array_layout");
		NewProperty("max_array_size");
		break;
	case STD_MSG_FLOAT32 :
		NewOutput("output_float32");
		break;

	case STD_MSG_FLOAT32_ARRAY :
		NewOutput("output_float32_array");
		NewOutput("array_layout");
		NewProperty("max_array_size");
		break;
	case STD_MSG_FLOAT64 :
		NewOutput("output_float64");
		break;
	case STD_MSG_FLOAT64_ARRAY :
		NewOutput("output_float64_array");
		NewOutput("array_layout");
		NewProperty("max_array_size");
		break;
	default:
		ReportError("This topic is not supported yet.");
	}
}

void MAPSros_topic_subscriber::CreateIOsForSensorTopics(bool* ros_header_avail)
{
	*ros_header_avail = false;
	switch(m_message) 
    {
	case SENSOR_MSG_IMAGE :
		NewOutput("output_image");
		*ros_header_avail = true;
        break;
    case SENSOR_MSG_COMPRESSED_IMAGE:
        NewOutput("output_compressed_image");
        NewProperty("compressed_image_width");
        NewProperty("compressed_image_height");
        *ros_header_avail = true;
        break;
	case SENSOR_MSG_LASER_SCAN :
		NewOutput("output_laser_scan_ranges");
		NewOutput("output_laser_scan_intensities");
		NewOutput("output_laser_scan_info");
		NewProperty("laser_discard_out_or_range_data");
		*ros_header_avail = true;
		break;
	case SENSOR_MSG_JOY :
		NewOutput("output_joy_axes");
		NewOutput("output_joy_buttons");
		*ros_header_avail = true;
		break;
	case SENSOR_MSG_IMU :
		NewOutput("orientation_quaternion");
		NewOutput("angular_velocities");
		NewOutput("accelerations");
		*ros_header_avail = true;
		break;
	case SENSOR_MSG_POINT_CLOUD:
		NewOutput("output_point_cloud");
		NewOutput("output_point_cloud_channel_sizes");
		NewOutput("output_point_cloud_channels");
		*ros_header_avail = true;
		break;
	case SENSOR_MSG_POINT_CLOUD2:
		NewOutput("output_point_cloud_2_info");
		NewOutput("output_point_cloud_2_fields_names");
		NewOutput("output_point_cloud_2_fields_info");
		NewOutput("output_float32_array","output_point_cloud_2_xyz");
        NewProperty("max_nb_points");
		*ros_header_avail = true;
		break;
	case SENSOR_MSG_RANGE:
		NewOutput("output_float32","range");
		NewOutput("range_info");
		*ros_header_avail = true;
		break;
    case SENSOR_MSG_NAV_SAT_FIX:
        NewOutput("navsatfix_status");
        NewOutput("navsatfix_lla_pos");
        NewOutput("navsatfix_pos_cov");
        NewOutput("navsatfix_pos_cov_type");
        *ros_header_avail = true;
        break;
	default:
		ReportError("This topic is not supported yet.");
	}
}

void MAPSros_topic_subscriber::CreateIOsForGeomTopics(bool* ros_header_avail)
{
	*ros_header_avail = false;
	switch(m_message) 
    {
	case GEOM_MSG_POINT :
		NewOutput("output_float64_array","output_point");
		break;
	case GEOM_MSG_POSE :
		NewOutput("output_float64_array","output_point");
		NewOutput("output_float64_array","output_quaternion");
		break;
	case GEOM_MSG_POSE_STAMPED :
		NewOutput("output_float64_array","output_point");
		NewOutput("output_float64_array","output_quaternion");
		*ros_header_avail = true;
        break;
	case GEOM_MSG_TWIST :
		NewOutput("output_twist");
		break;
    case GEOM_MSG_TWIST_STAMPED:
        NewOutput("output_twist");
        *ros_header_avail = true;
        break;
	default:
		ReportError("This topic is not supported yet.");
	}
}

void MAPSros_topic_subscriber::CreateIOsForNavTopics(bool* ros_header_avail)
{
    *ros_header_avail = false;
    switch(m_message) 
    {
    case NAV_MSG_ODOMETRY :
        NewOutput("output_float64_array","output_pose_position");
        NewOutput("output_float64_array","output_pose_orientation");
        NewOutput("output_covariance","outpout_pose_covariance");
        NewOutput("output_twist");
        NewOutput("output_covariance","output_twist_covariance");
        *ros_header_avail = true;
        break;
    default:
        ReportError("This topic is not supported yet.");
    }
}

void MAPSros_topic_subscriber::CreateIOsForVisuTopics(bool* ros_header_avail)
{
    *ros_header_avail = false;
    switch(m_message) 
    {
        case VISU_MSG_MARKER :
        case VISU_MSG_MARKER_ARRAY :
            NewOutput("output_marker_arrow");
            NewOutput("output_marker_cube");
            NewOutput("output_marker_sphere");
            NewOutput("output_marker_cylinder");
            NewOutput("output_marker_line_strip");
            NewOutput("output_marker_line_list");
            NewOutput("output_marker_cube_list");
            NewOutput("output_marker_sphere_list");
            NewOutput("output_marker_points");
            NewOutput("output_marker_text_view_facing");
            NewOutput("output_marker_mesh_resource");
            NewOutput("output_marker_triangle_list");
            *ros_header_avail = true;
            break;

        default:
            ReportError("This topic is not supported yet.");
    }
}

void MAPSros_topic_subscriber::CreateIOsForCANTopics(bool* ros_header_avail)
{
    *ros_header_avail = true;
    NewOutput("can_frames");
}

void MAPSros_topic_subscriber::CreateIOsForRmpTopics(bool* ros_header_avail)
{
    *ros_header_avail = true;
    switch(m_message)
    {
        case RMP_MSG_MOTOR_STATUS:
            NewOutput("output_motor_status_current");
            NewOutput("output_motor_status_current_limit");
            NewOutput("output_motor_status_mcu_inst_power");
            NewOutput("output_motor_status_mcu_total_energy");
            NewOutput("output_motor_status_max_current");
            NewOutput("output_motor_status_min_current_limit");
            break;
        case RMP_MSG_BATTERY:
            NewOutput("output_battery_charge_state");
            NewOutput("output_battery_temperature");
            NewOutput("output_battery_min_propulsion_charge_state");
            NewOutput("output_battery_aux_battery_voltage");
            NewOutput("output_battery_aux_battery_current");
            NewOutput("output_battery_abb_system_status");
            NewOutput("output_battery_aux_battery_status");
            break;
        case RMP_MSG_FAULT_STATUS:
            NewOutput("output_fault_status");
            break;
//        case RMP_MSG_BOOL_STAMPED:
//            NewOutput("output_bool");
//            break;
//        case RMP_MSG_AUDIO_COMMAND:
//            NewOutput("output_audio_command_command");
//            break;
        default:
            ReportError("This topic is not supported yet.");
    }
}

void MAPSros_topic_subscriber::Birth()
{
	m_first_time = true;
	if (m_ros_header_avail)
		m_transfer_ros_timestamp = GetBoolProperty("transfer_ROS_timestamps");

    m_n = NULL;
    m_sub = NULL;

    m_n = MAPSRosUtils::GetROSNode();
    if (m_n == NULL)
        Error("Could not create ROS node handle.");


    m_sub = new ros::Subscriber();
	if (m_sub == NULL)
		Error("Could not create ROS Subscriber.");


    int queue_size = (int)GetIntegerProperty("subscribe_queue_size");

	MAPSString topic_name = GetStringProperty("topic_name");

	switch(m_topic_type) 
    {
	case TOPIC_TYPE_STD:
		switch(m_message) 
        {
		case STD_MSG_TEXT:
			m_buffsize_out = (int)GetIntegerProperty("max_text_length");
			Output(0).AllocOutputBuffer(m_buffsize_out + 1);
            //{
                //ros::SubscribeOptions so =ros::SubscribeOptions::create<std_msgs::String>((const char*)topic_name,queue_size == -1? 1000:queue_size,boost::bind(&MAPSros_topic_subscriber::ROSStringReceivedCallback,this,_1),ros::VoidPtr(),&_cb_queue);
                //sopts.init<std_msgs::String>((const char*)topic_name,queue_size == -1?1000:queue_size,);
                //*_sub = _n->subscribe(so);
            //}
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSStringReceivedCallback,this);
			break;
		case STD_MSG_INT32:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSInt32ReceivedCallback,this);
			break;
		case STD_MSG_INT32_ARRAY:
			m_buffsize_out = (int)GetIntegerProperty("max_array_size");
			Output(0).AllocOutputBuffer(m_buffsize_out);
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSInt32ArrayReceivedCallback,this);
			break;
		case STD_MSG_INT64:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSInt64ReceivedCallback,this);
			break;
		case STD_MSG_INT64_ARRAY:
			m_buffsize_out = (int)GetIntegerProperty("max_array_size");
			Output(0).AllocOutputBuffer(m_buffsize_out);
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSInt64ArrayReceivedCallback,this);
			break;
		case STD_MSG_FLOAT32:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSFloat32ReceivedCallback,this);
			break;
		case STD_MSG_FLOAT32_ARRAY:
			m_buffsize_out = (int)GetIntegerProperty("max_array_size");
			Output(0).AllocOutputBuffer(m_buffsize_out);
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSFloat32ArrayReceivedCallback,this);
			break;
		case STD_MSG_FLOAT64:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSFloat64ReceivedCallback,this);
			break;
		case STD_MSG_FLOAT64_ARRAY:
			m_buffsize_out = (int)GetIntegerProperty("max_array_size");
			Output(0).AllocOutputBuffer(m_buffsize_out);
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSFloat64ArrayReceivedCallback,this);
			break;
		default:
			{
				MAPSStreamedString ss;
				ss << "Topic " << GetStringProperty("topic_type") << "/" << GetStringProperty("message") << " not supported yet.";
				Error(ss);
			}
		}
		break;
	case TOPIC_TYPE_SENSOR:
		switch(m_message) 
        {
		case SENSOR_MSG_IMAGE:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?16:queue_size,
                                  &MAPSros_topic_subscriber::ROSImageReceivedCallback, this);
			break;
        case SENSOR_MSG_COMPRESSED_IMAGE:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?16:queue_size,
                                  &MAPSros_topic_subscriber::ROSCompressedImageReceivedCallback, this);
            break;
		case SENSOR_MSG_LASER_SCAN:
			m_discard_out_of_range = GetBoolProperty("laser_discard_out_or_range_data");
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?64:queue_size, &MAPSros_topic_subscriber::ROSLaserScanReceivedCallback,this);
			break;
		case SENSOR_MSG_JOY:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSJoyReceivedCallback,this);
			break;
		case SENSOR_MSG_IMU:
//        {
//            ros::SubscribeOptions so =ros::SubscribeOptions::create<sensor_msgs::Imu>((const char*)topic_name,queue_size == -1? 1000:queue_size,boost::bind(&MAPSros_topic_subscriber::ROSImuReceivedCallback,this,_1),ros::VoidPtr(),&_cb_queue);
//            //sopts.init<std_msgs::String>((const char*)topic_name,queue_size == -1?1000:queue_size,);
//            *_sub = _n->subscribe(so);
//        }
            *m_sub = m_n->subscribe((const char*)topic_name, 1000, &MAPSros_topic_subscriber::ROSImuReceivedCallback,this);
			break;
		case SENSOR_MSG_POINT_CLOUD:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?queue_size == -1?16:queue_size:queue_size, &MAPSros_topic_subscriber::ROSPointCloudReceivedCallback,this);
			break;
		case SENSOR_MSG_POINT_CLOUD2:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?queue_size == -1?16:queue_size:queue_size, &MAPSros_topic_subscriber::ROSPointCloud2ReceivedCallback,this);
			break;
		case SENSOR_MSG_RANGE:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?queue_size == -1?1000:queue_size:queue_size, &MAPSros_topic_subscriber::ROSRangeReceivedCallback,this);
			break;
        case SENSOR_MSG_NAV_SAT_FIX:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?queue_size == -1?1000:queue_size:queue_size, &MAPSros_topic_subscriber::ROSNavSatFixReceivedCallback,this);
            break;
		default:
			{
				MAPSStreamedString ss;
				ss << "Topic " << GetStringProperty("topic_type") << "/" << GetStringProperty("message") << " not supported yet.";
				Error(ss);
			}
		}
		break;
	case TOPIC_TYPE_GEOM:
		switch(m_message) 
        {
		case GEOM_MSG_POINT:
			Output(0).AllocOutputBuffer(3);
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSPointReceivedCallback,this);
			break;
		case GEOM_MSG_POSE:
			Output(0).AllocOutputBuffer(3);
			Output(1).AllocOutputBuffer(4);
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSPoseReceivedCallback,this);
			break;
		case GEOM_MSG_POSE_STAMPED:
			Output(0).AllocOutputBuffer(3);
			Output(1).AllocOutputBuffer(4);
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSPoseStampedReceivedCallback,this);
			break;
		case GEOM_MSG_TWIST:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSTwistReceivedCallback,this);
			break;
        case GEOM_MSG_TWIST_STAMPED:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSTwistStampedReceivedCallback,this);
            break;
		default:
			{
				MAPSStreamedString ss;
				ss << "Topic " << GetStringProperty("topic_type") << "/" << GetStringProperty("message") << " not supported yet.";
				Error(ss);
			}
		}
		break;
    case TOPIC_TYPE_NAV:
        switch(m_message) 
        {
        case NAV_MSG_ODOMETRY:
            Output(0).AllocOutputBuffer(3); //Pose point
            Output(1).AllocOutputBuffer(4); //Pose quaternion
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSOdometryReceivedCallback,this);
            break;
        default:
            {
                MAPSStreamedString ss;
                ss << "Topic " << GetStringProperty("topic_type") << "/" << GetStringProperty("message") << " not supported yet.";
                Error(ss);
            }
        }
        break;
    case TOPIC_TYPE_VISU:
        switch(m_message) 
        {
        case VISU_MSG_MARKER:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSVisuMarkerReceivedCallback,this);
            break;
        case VISU_MSG_MARKER_ARRAY:
            *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSVisuMarkerArrayReceivedCallback,this);
            break;
        default:
            {
                MAPSStreamedString ss;
                ss << "Topic " << GetStringProperty("topic_type") << "/" << GetStringProperty("message") << " not supported yet.";
                Error(ss);
            }
            break;
        }
        break;
    case TOPIC_TYPE_CAN:
        *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSCANFrameReceivedCallback, this );
        break;

    case TOPIC_TYPE_RMP:

        switch(m_message)
        {
            case RMP_MSG_MOTOR_STATUS:
                *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSRmpMotorStatusReceivedCallback,this);
                break;
            case RMP_MSG_BATTERY:
                *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSRmpBatteryReceivedCallback,this);
                break;
            case RMP_MSG_FAULT_STATUS:
                *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSRmpFaultStatusReceivedCallback,this);
                break;
//            case RMP_MSG_BOOL_STAMPED:
//                *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSRMPBoolStampedReceivedCallback,this);
//                break;
//            case RMP_MSG_AUDIO_COMMAND:
//                *m_sub = m_n->subscribe((const char*)topic_name, queue_size == -1?1000:queue_size, &MAPSros_topic_subscriber::ROSRMPAudioCommandReceivedCallback,this);
//                break;
            default:
            {
                MAPSStreamedString ss;
                ss << "Topic " << GetStringProperty("topic_type") << "/" << GetStringProperty("message") << " not supported yet.";
                Error(ss);
            }
                break;
        }
        break;

	}

	if (m_sub == NULL) 
    {
		MAPSStreamedString ss;
		ss << "Could not subscribe to topic " << topic_name << " (type: " << GetStringProperty("topic_type") << "/" << GetStringProperty("message");
		Error(ss);
	}

}

void MAPSros_topic_subscriber::ROSStringReceivedCallback(const std_msgs::StringConstPtr& message)
{
    try 
    {
    MAPSIOElt* ioeltout = StartWriting(Output(0));
    char* data_out = ioeltout->TextAscii();
	int txt_len = message->data.length();
	if (txt_len > m_buffsize_out) 
    {
		txt_len = m_buffsize_out;
		MAPSStreamedString ss;
		ss << "Received message too long (length = " << txt_len << "). You should increase the max_text_length property. Truncating...";
		ReportWarning(ss);
	}

    MAPS::Memcpy(data_out,message->data.c_str(),txt_len);
	ioeltout->TextAscii()[txt_len] = '\0';

	ioeltout->VectorSize() = txt_len;
	StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::SetComponentInError()
{
    // filter out RTMAPS exceptions because ROS1 can not catch them.
    try 
    {
        Error("setting component in error.");
    } 
    catch(...) {}

    CommitSuicide();
}

void MAPSros_topic_subscriber::ROSImageReceivedCallback(const sensor_msgs::Image::ConstPtr& ros_image)
{
    try 
    {
        if (m_first_time) 
        {
            m_first_time = false;
            MAPSUInt32 chanseq, depth = 8;
            if (ros_image->encoding == "rgb8")
                chanseq = MAPS_CHANNELSEQ_RGB;
            else if (ros_image->encoding == "bgr8")
                chanseq = MAPS_CHANNELSEQ_BGR;
            else if (ros_image->encoding == "mono8")
                chanseq = MAPS_CHANNELSEQ_GRAY;
            else if (ros_image->encoding == "mono16") 
            {
                chanseq = MAPS_CHANNELSEQ_GRAY;
                depth = 16;
            } 
            else if (ros_image->encoding == "rgba8")
                chanseq = MAPS_CHANNELSEQ_RGBA;
            else if (ros_image->encoding == "bgra8")
                chanseq = MAPS_CHANNELSEQ_BGRA;
            else 
            {
                if (m_sub)
                    m_sub->shutdown(); // stop topic subscription
                MAPSStreamedString ss;
                ss << "Image format not supported: " << (const char *) ros_image->encoding.c_str();
                ReportError(ss);
                SetComponentInError();
                return;
            }
            IplImage model = MAPS::IplImageModel(ros_image->width, ros_image->height, chanseq, 0, depth);
            Output(0).AllocOutputBufferIplImage(model);
        }
        MAPSIOElt *ioeltout = StartWriting(Output(0));
        IplImage &image_out = ioeltout->IplImage();

        int size = MIN((unsigned int) image_out.imageSize, ros_image->data.size());
        MAPS::Memcpy(image_out.imageData, (char *) &ros_image->data[0], size);

        if (m_transfer_ros_timestamp) 
        {
            ioeltout->Timestamp() = MAPSRosUtils::ROSTimeToMAPSTimestamp(ros_image->header.stamp);
        }
        StopWriting(ioeltout);
    } 
    catch(int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSCompressedImageReceivedCallback(const sensor_msgs::CompressedImage::ConstPtr& ros_image)
{
    try 
    {
        int32_t width = GetIntegerProperty("compressed_image_width");
        int32_t height = GetIntegerProperty("compressed_image_height");

        if (m_first_time)
        {
            m_first_time = false;
            MAPSUInt32 encoding = MAPS_IMAGECODING_UNKNOWN;
            if (ros_image->format.find("jpeg")  != std::string::npos)
                encoding = MAPS_IMAGECODING_JPEG;
            else if (ros_image->format.find("png") != std::string::npos)
                encoding = MAPS_IMAGECODING_PNG;
            else 
            {
                if (m_sub)
                    m_sub->shutdown(); // stop topic subscription
                MAPSStreamedString ss;
                ss << "Image format not supported: " << (const char *) ros_image->format.c_str();
                ReportError(ss);
                SetComponentInError();
                return;
            }
            int32_t size = MAX((int32_t)ros_image->data.size(), (int32_t)(width * height * 4));

            MAPSStreamedString ss;
            ss << "Max output image size : " << size << ": " << width << "x" << height;
            ss << ", image encoding: " << ros_image->format.c_str();
            ReportInfo(ss);

            Output(0).AllocOutputBufferMAPSImage(size, width, height, encoding);
        }
        int32_t size = MIN((int32_t)ros_image->data.size(), width * height * 4);

        MAPSIOElt *ioeltout = StartWriting(Output(0));
        MAPSImage &image_out = ioeltout->MAPSImage();
        MAPS::Memcpy(image_out.imageData, (char *) &ros_image->data[0], size);
        image_out.imageSize = (int32_t)ros_image->data.size();
        if (m_transfer_ros_timestamp) 
        {
            ioeltout->Timestamp() = MAPSRosUtils::ROSTimeToMAPSTimestamp(ros_image->header.stamp);
        }
        StopWriting(ioeltout);
    } 
    catch(int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSInt32ReceivedCallback(const std_msgs::Int32ConstPtr& msg)
{
    try 
    {
    MAPSIOElt* ioeltout = StartWriting(Output(0));
	ioeltout->Integer32() = msg->data;
	StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }

}

void MAPSros_topic_subscriber::ROSInt32ArrayReceivedCallback(const std_msgs::Int32MultiArrayConstPtr& msg)
{
    try 
    {
    MAPSTimestamp t = MAPS::CurrentTime();

	MAPSIOElt* ioeltout = StartWriting(Output(0));
	int vectorsize_out = msg->data.size();

	if (vectorsize_out > m_buffsize_out)  
    {
		MAPSStreamedString ss;
		ss << "Received array is too big (size = " << vectorsize_out << "). Increase the max array size property. Truncating...";
		ReportWarning(ss);
		vectorsize_out = m_buffsize_out;
	}
	for (int i=0; i<vectorsize_out;i++) 
    {
		ioeltout->Integer32(i) = msg->data[i];
	}
	ioeltout->Timestamp() = t;
	ioeltout->VectorSize() = vectorsize_out;
	StopWriting(ioeltout);

	const std_msgs::MultiArrayLayout* layout_ptr = &(msg->layout);
	OutputArrayLayout(layout_ptr,t);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSInt64ReceivedCallback(const std_msgs::Int64ConstPtr& msg)
{
    try 
    {
    MAPSIOElt* ioeltout = StartWriting(Output(0));

	ioeltout->Integer64() = msg->data;

	StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}
void MAPSros_topic_subscriber::ROSInt64ArrayReceivedCallback(const std_msgs::Int64MultiArrayConstPtr& msg)
{
    try 
    {
    MAPSTimestamp t = MAPS::CurrentTime();

	MAPSIOElt* ioeltout = StartWriting(Output(0));
	int vectorsize_out = msg->data.size();

	if (vectorsize_out > m_buffsize_out)  
    {
		MAPSStreamedString ss;
		ss << "Received array is too big (size = " << vectorsize_out << "). Increase the max array size property. Truncating...";
		ReportWarning(ss);
		vectorsize_out = m_buffsize_out;
	}
	for (int i=0; i<vectorsize_out;i++) 
    {
		ioeltout->Integer64(i) = msg->data[i];
	}
	ioeltout->Timestamp() = t;
	ioeltout->VectorSize() = vectorsize_out;
	StopWriting(ioeltout);

	const std_msgs::MultiArrayLayout* layout_ptr = &(msg->layout);
	OutputArrayLayout(layout_ptr,t);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSFloat32ReceivedCallback(const std_msgs::Float32ConstPtr& msg)
{
    try {
    MAPSIOElt* ioeltout = StartWriting(Output(0));
	ioeltout->Float32() = msg->data;
	StopWriting(ioeltout);
    } catch (int error) {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSFloat32ArrayReceivedCallback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    try 
    {
    MAPSTimestamp t = MAPS::CurrentTime();

	MAPSIOElt* ioeltout = StartWriting(Output(0));
	int vectorsize_out = msg->data.size();

	if (vectorsize_out > m_buffsize_out)  
    {
		MAPSStreamedString ss;
		ss << "Received array is too big (size = " << vectorsize_out << "). Increase the max array size property. Truncating...";
		ReportWarning(ss);
		vectorsize_out = m_buffsize_out;
	}
	for (int i=0; i<vectorsize_out;i++) 
    {
		ioeltout->Float32(i) = msg->data[i];
	}
	ioeltout->Timestamp() = t;
	ioeltout->VectorSize() = vectorsize_out;
	StopWriting(ioeltout);

	const std_msgs::MultiArrayLayout* layout_ptr = &(msg->layout);
	OutputArrayLayout(layout_ptr,t);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}
void MAPSros_topic_subscriber::ROSFloat64ReceivedCallback(const std_msgs::Float64ConstPtr& msg)
{
    try 
    {
    MAPSIOElt* ioeltout = StartWriting(Output(0));
	ioeltout->Float64() = msg->data;
	StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSFloat64ArrayReceivedCallback(const std_msgs::Float64MultiArrayConstPtr& msg)
{
    try 
    {
    MAPSTimestamp t = MAPS::CurrentTime();

	MAPSIOElt* ioeltout = StartWriting(Output(0));
	int vectorsize_out = msg->data.size();

	if (vectorsize_out > m_buffsize_out)  
    {
		MAPSStreamedString ss;
		ss << "Received array is too big (size = " << vectorsize_out << "). Increase the max array size property. Truncating...";
		ReportWarning(ss);
		vectorsize_out = m_buffsize_out;
	}
	for (int i=0; i<vectorsize_out;i++) 
    {
		ioeltout->Float64(i) = msg->data[i];
	}
	ioeltout->Timestamp() = t;
	ioeltout->VectorSize() = vectorsize_out;
	StopWriting(ioeltout);

	const std_msgs::MultiArrayLayout* layout_ptr = &(msg->layout);
	OutputArrayLayout(layout_ptr,t);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::OutputArrayLayout(const std_msgs::MultiArrayLayout* ros_layout, MAPSTimestamp t)
{
    try 
    {
    MAPSIOElt* ioeltout_layout = StartWriting(Output(1));
	ROSArrayLayout* maps_layout = (ROSArrayLayout*)ioeltout_layout->Data();

	int nb_dims = ros_layout->dim.size();
	if (nb_dims > MAX_ARRAY_LAYOUT_DIMENSIONS) 
    {
		MAPSStreamedString ss;
		ss << "Received array has too many dimensions (" << nb_dims <<"). This component supports " << MAX_ARRAY_LAYOUT_DIMENSIONS << " dimensions max on array_layout output.Truncating array_layout info output.";
		ReportWarning(ss);
		nb_dims = MAX_ARRAY_LAYOUT_DIMENSIONS;
	}
	maps_layout->nb_dims = nb_dims;
	maps_layout->data_offset = ros_layout->data_offset;
	for (int i=0; i<nb_dims; i++) 
    {
		MAPS::Memcpy(maps_layout->dim[i].label,ros_layout->dim[i].label.c_str(),ros_layout->dim[i].label.length());
		maps_layout->dim[i].label[ros_layout->dim[i].label.length()] = '\0';
		maps_layout->dim[i].size = ros_layout->dim[i].size;
		maps_layout->dim[i].stride = ros_layout->dim[i].stride;
	}

	ioeltout_layout->Timestamp() = t;
	ioeltout_layout->VectorSize() = sizeof(ROSArrayLayout);
	StopWriting(ioeltout_layout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSLaserScanReceivedCallback(const sensor_msgs::LaserScan::ConstPtr& ros_laser_scan)
{
    try 
    {
    MAPSTimestamp t;
	if (false == m_transfer_ros_timestamp)
		t = MAPS::CurrentTime();
	else
        t = MAPSRosUtils::ROSTimeToMAPSTimestamp(ros_laser_scan->header.stamp);

	if (m_first_time) 
    {
		m_first_time = false;
		m_nb_laser_scan_points = ros_laser_scan->ranges.size();
		m_nb_laser_intens_data = ros_laser_scan->intensities.size();
		Output(0).AllocOutputBuffer(m_nb_laser_scan_points);
		if (m_nb_laser_intens_data > 0) 
        {
			Output(1).AllocOutputBuffer(m_nb_laser_intens_data);
		}
	}

	MAPSIOElt* out_ranges = NULL, *out_intens = NULL, *out_infos = NULL;
	out_ranges = StartWriting(Output(0));
	if (m_nb_laser_intens_data) 
    {
		out_intens = StartWriting(Output(1));
	}
	out_infos = StartWriting(Output(2));
    MAPSFloat64* ranges_data = &out_ranges->Float64();

	int nb_points = ros_laser_scan->ranges.size();
	if (nb_points > m_nb_laser_scan_points) 
    {
		MAPSStreamedString ss;
		ss << "Number of scan points has increased. Problem. Truncating.";
		ReportWarning(ss);
		nb_points = m_nb_laser_scan_points;
	}

    MAPSFloat64* intens_data = NULL;
	if (m_nb_laser_intens_data)
		intens_data = &out_intens->Float64();
	int vectorsize_out = 0;
    MAPSFloat64 range;
	for (int i=0; i<nb_points; i++) 
    {
		range = ros_laser_scan->ranges[i];
		if (m_discard_out_of_range) 
        {
			if (range < ros_laser_scan->range_min || range > ros_laser_scan->range_max)
				continue;
		}
		*(ranges_data++) = ros_laser_scan->ranges[i];
		if (m_nb_laser_intens_data) 
        {
			*(intens_data++) = ros_laser_scan->intensities[i];
		}
		vectorsize_out++;
	}
	out_infos->Float64(0) = ros_laser_scan->angle_min;
	out_infos->Float64(1) = ros_laser_scan->angle_max;
	out_infos->Float64(2) = ros_laser_scan->angle_increment;
	out_infos->Float64(3) = ros_laser_scan->time_increment;
	out_infos->Float64(4) = ros_laser_scan->scan_time;
	out_infos->Float64(5) = ros_laser_scan->range_min;
	out_infos->Float64(6) = ros_laser_scan->range_max;

	out_ranges->VectorSize() = vectorsize_out;
	if (m_nb_laser_intens_data) 
    {
		out_intens->VectorSize() = vectorsize_out;
	}

	out_infos->Timestamp() = t;
	out_ranges->Timestamp() = t;
	if (m_nb_laser_intens_data) 
    {
		out_intens->Timestamp() = t;
	}

	StopWriting(out_infos);
	if (m_nb_laser_intens_data) 
    {
		StopWriting(out_intens);
	}
	StopWriting(out_ranges);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSPointCloudReceivedCallback(const sensor_msgs::PointCloud::ConstPtr& ros_point_cloud)
{
    try 
    {
    MAPSTimestamp t;
	if (false == m_transfer_ros_timestamp)
		t = MAPS::CurrentTime();
	else
        t = MAPSRosUtils::ROSTimeToMAPSTimestamp(ros_point_cloud->header.stamp);

	if (m_first_time) 
    {
		m_first_time = false;
		m_nb_points = ros_point_cloud->points.size();
		m_nb_channels = ros_point_cloud->channels.size();

		Output(0).AllocOutputBuffer(m_nb_points*3);
		if (m_nb_channels > 0) 
        {
			Output(1).AllocOutputBuffer(m_nb_channels);
			Output(2).AllocOutputBuffer(m_nb_points);
		}
	}

	MAPSIOElt* out_points_xyz = NULL, *out_nb_channels = NULL, *out_distances = NULL;
	out_points_xyz = StartWriting(Output(0));
	if (m_nb_channels) 
    {
		out_nb_channels = StartWriting(Output(1));
		out_distances = StartWriting(Output(2));
	}
	MAPSFloat32* xyz_points = &out_points_xyz->Float32();

	int nb_points = ros_point_cloud->points.size();
	if (nb_points > m_nb_points) 
    {
		MAPSStreamedString ss;
		ss << "Number of scan points has increased. Problem. Truncating.";
		ReportWarning(ss);
		nb_points = m_nb_points;
	}

	for (int i=0; i<nb_points; i++) 
    {
		*(xyz_points++) = ros_point_cloud->points[i].x;
		*(xyz_points++) = ros_point_cloud->points[i].y;
		*(xyz_points++) = ros_point_cloud->points[i].z;
	}
	out_points_xyz->VectorSize() = nb_points*3;

	MAPSFloat32* channels_data = NULL;
	MAPSInt32* channels_sizes = NULL;
	if (m_nb_channels) 
    {
		channels_sizes = &out_nb_channels->Integer32();
		channels_data = &out_distances->Float32();
	}
	for (int i=0; i<m_nb_channels; i++) 
    {
		int nb_ranges_on_channel = ros_point_cloud->channels[i].values.size();
		*(channels_sizes++) = nb_ranges_on_channel;
		for (int j=0; j<nb_ranges_on_channel; j++) {
			*(channels_data++) = ros_point_cloud->channels[i].values[j];
		}
	}

	if (m_nb_channels) 
    {
		out_nb_channels->VectorSize() = m_nb_channels;
		out_distances->VectorSize() = nb_points;
		out_nb_channels->Timestamp() = t;
		out_distances->Timestamp() = t;
		StopWriting(out_nb_channels);
		StopWriting(out_distances);
	}
	out_points_xyz->Timestamp() = t;
	StopWriting(out_points_xyz);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSPointCloud2ReceivedCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_point_cloud2)
{
    try 
    {
    MAPSTimestamp t;
	if (false == m_transfer_ros_timestamp)
		t = MAPS::CurrentTime();
	else
        t = MAPSRosUtils::ROSTimeToMAPSTimestamp(ros_point_cloud2->header.stamp);

	if (m_first_time) 
    {
		m_first_time = false;
		m_nb_fields = ros_point_cloud2->fields.size();
        m_point_step = ros_point_cloud2->point_step;
		m_nb_points = ros_point_cloud2->data.size()/ros_point_cloud2->point_step;
        int max_nb_points = (int)GetIntegerProperty("max_nb_points");
		if (m_nb_fields > 0) 
        {
			Output(1).AllocOutputBuffer(m_nb_fields*256); //255 chars max per field name.
			Output(2).AllocOutputBuffer(m_nb_fields*3); //(offset, datatype, count)
		}
        if (max_nb_points > 0) 
        {
            if (m_nb_points > max_nb_points) 
            {
                MAPSStreamedString ss;
                ss << "Number of points if first packet (" << m_nb_points << ") is higher than max_nb_points property (" << max_nb_points << "). Using number of points in first packet received.";
                ReportWarning(ss);
                max_nb_points = m_nb_points;
            }
            m_nb_points = max_nb_points;
        }
        Output(3).AllocOutputBuffer(m_nb_points * 3); //data
	}

	MAPSIOElt* out_info = NULL, *out_field_names = NULL, *out_fields_info = NULL, *out_data = NULL;
	out_info = StartWriting(Output(0));
	if (m_nb_fields) 
    {
		out_field_names = StartWriting(Output(1));
		out_fields_info = StartWriting(Output(2));
	}
	out_data = StartWriting(Output(3));

	out_info->Integer32(0) = ros_point_cloud2->width;
	out_info->Integer32(1) = ros_point_cloud2->height;
	out_info->Integer32(2) = ros_point_cloud2->is_bigendian ? 1 : 0;
	out_info->Integer32(3) = ros_point_cloud2->point_step;
	out_info->Integer32(4) = ros_point_cloud2->row_step;
	out_info->Integer32(5) = ros_point_cloud2->is_dense ? 1 : 0;

	int nb_fields = ros_point_cloud2->fields.size();
	if (nb_fields > m_nb_fields) 
    {
		MAPSStreamedString ss;
		ss << "Number of fields has increased. Problem. Truncating.";
		ReportWarning(ss);
		nb_fields = m_nb_fields;
	}
	if (nb_fields != 3) 
    {
		if (nb_fields < 3) 
        {
			MAPSStreamedString ss;
			ss << "So far, this components supports pointcloud2 messages with at least 3 fields 'x','y' and 'z'.";
			ss << " Only " << nb_fields << " have been found.";
            ReportError(ss);
            if (m_sub)
                m_sub->shutdown(); // stop topic subscription
            SetComponentInError();
            return;
        } 
        else if (nb_fields > 3) 
        {
            MAPSStreamedString ss;
            ss << "Found more than 3 fields int pointcloud2 message. Only x,y,z fields will be taken into account on the point cloud output.";
            ReportWarning(ss);
        }
	}
	MAPSString field_names;
	for (int i=0; i<nb_fields; i++) 
    {
		field_names += (const char*)ros_point_cloud2->fields[i].name.c_str();
		field_names += "|";
	}
	if (field_names.Len() >= 255*nb_fields) 
    {
		MAPSStreamedString ss;
		ss << "Field names too long. Problem. Truncating.";
		ReportWarning(ss);
		field_names = field_names.Left(255*nb_fields);
	}
	MAPS::Strcpy(out_field_names->TextAscii(),(const char*)field_names);
	out_field_names->VectorSize() = field_names.Len();

	for (int i=0; i<nb_fields; i++) 
    {
		out_fields_info->Integer32(3*i) = ros_point_cloud2->fields[i].offset;
		out_fields_info->Integer32(3*i+1) = ros_point_cloud2->fields[i].datatype;
		out_fields_info->Integer32(3*i+2) = ros_point_cloud2->fields[i].count;
	}
	out_fields_info->VectorSize() = nb_fields*3;

    int nb_points = ros_point_cloud2->data.size() / ros_point_cloud2->point_step;
    if (nb_points > m_nb_points) 
    {
		MAPSStreamedString ss;
        ss << "Amount of points (" << nb_points << ") exceeds maximum number of points (" << m_nb_points << "). Truncating. If property max_nb_points is set to -1, it means that number of points has increased since first sample. Consider increasing max_nb_points.";
		ReportWarning(ss);
        nb_points = m_nb_points;
	}
    MAPSFloat32* data_out = &out_data->Float32();
    if (m_point_step == 12) 
    {
        MAPS::Memcpy(data_out,&(ros_point_cloud2->data[0]),nb_points * 4 * 3);
    } 
    else 
    {
        MAPSFloat32* data_in = (MAPSFloat32*) &(ros_point_cloud2->data[0]);
        for (int i=nb_points; i>0; --i) 
        {
            *(data_out++) = *(data_in++);
            *(data_out++) = *(data_in++);
            *(data_out++) = *(data_in++);
            data_in += ros_point_cloud2->point_step/4 - 3;
        }
    }
    out_data->VectorSize() = nb_points * 3;


	out_info->Timestamp() = t;
	out_field_names->Timestamp() = t;
	out_fields_info->Timestamp() = t;
	out_data->Timestamp() = t;

	StopWriting(out_data);
	StopWriting(out_fields_info,nb_fields==0);
	StopWriting(out_field_names,nb_fields==0);
	StopWriting(out_info);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSJoyReceivedCallback(const sensor_msgs::Joy::ConstPtr& ros_joy)
{
    try {
    MAPSTimestamp t;
	if (false == m_transfer_ros_timestamp)
		t = MAPS::CurrentTime();
	else
        t = MAPSRosUtils::ROSTimeToMAPSTimestamp(ros_joy->header.stamp);

	if (m_first_time) 
    {
		m_first_time = false;
		m_nb_joy_axes = ros_joy->axes.size();
		m_nb_joy_buttons = ros_joy->buttons.size();
		Output(0).AllocOutputBuffer(m_nb_joy_axes);
		Output(1).AllocOutputBuffer(m_nb_joy_buttons);	
	}

	MAPSIOElt* out_axes, *out_buttons;
	out_axes = StartWriting(Output(0));
	for(int i=0;i<m_nb_joy_axes;i++)
		out_axes->Float32(i) = ros_joy->axes.at(i);
	out_axes->Timestamp() = t;
	StopWriting(out_axes);

	out_buttons = StartWriting(Output(1));
	for(int i=0;i<m_nb_joy_buttons;i++)
		out_buttons->Integer32(i) = ros_joy->buttons.at(i);
	out_buttons->Timestamp() = t;
	StopWriting(out_buttons);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSImuReceivedCallback(const sensor_msgs::Imu::ConstPtr& ros_imu)
{
    try 
    {
    MAPSTimestamp t;
	if (false == m_transfer_ros_timestamp)
		t = MAPS::CurrentTime();
	else
        t = MAPSRosUtils::ROSTimeToMAPSTimestamp(ros_imu->header.stamp);

	MAPSIOElt* out_orientation, *out_gyro, *out_acc;
	out_orientation = StartWriting(Output(0));
	out_gyro = StartWriting(Output(1));
	out_acc = StartWriting(Output(2));
	out_orientation->Float64(0) = ros_imu->orientation.x;
	out_orientation->Float64(1) = ros_imu->orientation.y;
	out_orientation->Float64(2) = ros_imu->orientation.z;
	out_orientation->Float64(3) = ros_imu->orientation.w;
	
	out_gyro->Float64(0) = ros_imu->angular_velocity.x;
	out_gyro->Float64(1) = ros_imu->angular_velocity.y;
	out_gyro->Float64(2) = ros_imu->angular_velocity.z;
	
	out_acc->Float64(0) = ros_imu->linear_acceleration.x;
	out_acc->Float64(1) = ros_imu->linear_acceleration.y;
	out_acc->Float64(2) = ros_imu->linear_acceleration.z;
	
	out_orientation->Timestamp() = t;
	out_gyro->Timestamp() = t;
	out_acc->Timestamp() = t;
	StopWriting(out_acc);
	StopWriting(out_gyro);
	StopWriting(out_orientation);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSPointReceivedCallback(const geometry_msgs::Point::ConstPtr& point)
{
    try 
    {
    MAPSIOElt* ioeltout = StartWriting(Output(0));
	ioeltout->Float64(0) = point->x;
	ioeltout->Float64(1) = point->y;
	ioeltout->Float64(2) = point->z;
	StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSPoseReceivedCallback(const geometry_msgs::Pose::ConstPtr& pose)
{
    try 
    {
    MAPSTimestamp t = MAPS::CurrentTime();
	MAPSIOElt* ioeltout = StartWriting(Output(0));
	ioeltout->Float64(0) = pose->position.x;
	ioeltout->Float64(1) = pose->position.y;
	ioeltout->Float64(2) = pose->position.z;
	ioeltout->Timestamp() = t;
	StopWriting(ioeltout);
	MAPSIOElt* ioeltout2 = StartWriting(Output(1));
	ioeltout2->Float64(0) = pose->orientation.x;
	ioeltout2->Float64(1) = pose->orientation.y;
	ioeltout2->Float64(2) = pose->orientation.z;
	ioeltout2->Float64(3) = pose->orientation.w;
	ioeltout2->Timestamp() = t;
	StopWriting(ioeltout2);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSPoseStampedReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& posestamped)
{
    try 
    {
    MAPSTimestamp t;
	if (false == m_transfer_ros_timestamp)
		t = MAPS::CurrentTime();
	else
        t = MAPSRosUtils::ROSTimeToMAPSTimestamp(posestamped->header.stamp);

	MAPSIOElt* ioeltout = StartWriting(Output(0));
	ioeltout->Float64(0) = posestamped->pose.position.x;
	ioeltout->Float64(1) = posestamped->pose.position.y;
	ioeltout->Float64(2) = posestamped->pose.position.z;
	ioeltout->Timestamp() = t;
	StopWriting(ioeltout);
	MAPSIOElt* ioeltout2 = StartWriting(Output(1));
	ioeltout2->Float64(0) = posestamped->pose.orientation.x;
	ioeltout2->Float64(1) = posestamped->pose.orientation.y;
	ioeltout2->Float64(2) = posestamped->pose.orientation.z;
	ioeltout2->Float64(3) = posestamped->pose.orientation.w;
	ioeltout2->Timestamp() = t;
	StopWriting(ioeltout2);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSRangeReceivedCallback(const sensor_msgs::Range::ConstPtr& ros_range)
{
    try 
    {
    MAPSTimestamp t;
	if (false == m_transfer_ros_timestamp)
		t = MAPS::CurrentTime();
	else
        t = MAPSRosUtils::ROSTimeToMAPSTimestamp(ros_range->header.stamp);

	MAPSIOElt* ioeltout = StartWriting(Output(0));
	ioeltout->Float32() = ros_range->range;
	ioeltout->Timestamp() = t;
	StopWriting(ioeltout);

	ioeltout = StartWriting(Output(1));
	ioeltout->Float64(0) = ros_range->radiation_type;
	ioeltout->Float64(1) = ros_range->field_of_view;
	ioeltout->Float64(2) = ros_range->min_range;
	ioeltout->Float64(3) = ros_range->max_range;
	ioeltout->Timestamp() = t;
	StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSNavSatFixReceivedCallback(const sensor_msgs::NavSatFix::ConstPtr& ros_navsatfix)
{
    try 
    {
    MAPSTimestamp t;
    if (false == m_transfer_ros_timestamp)
        t = MAPS::CurrentTime();
    else
        t = MAPSRosUtils::ROSTimeToMAPSTimestamp(ros_navsatfix->header.stamp);

    MAPSIOElt* ioeltout_status = StartWriting(Output(0));
    MAPSIOElt* ioeltout_pos_lla = StartWriting(Output(1));
    MAPSIOElt* ioeltout_pos_cov = StartWriting(Output(2));
    MAPSIOElt* ioeltout_pos_cov_type = StartWriting(Output(3));

    ioeltout_status->Integer32(0) = ros_navsatfix->status.status;
    ioeltout_status->Integer32(1) = ros_navsatfix->status.service;

    ioeltout_pos_lla->Float64(0) = ros_navsatfix->latitude;
    ioeltout_pos_lla->Float64(1) = ros_navsatfix->longitude;
    ioeltout_pos_lla->Float64(2) = ros_navsatfix->altitude;

    for (int i=0; i<9; i++) 
    {
        ioeltout_pos_cov->Float64(i) = ros_navsatfix->position_covariance[i];
    }

    ioeltout_pos_cov_type->Integer32() = ros_navsatfix->position_covariance_type;

    ioeltout_status->Timestamp() = t;
    ioeltout_pos_lla->Timestamp() = t;
    ioeltout_pos_cov->Timestamp() = t;
    ioeltout_pos_cov_type->Timestamp() = t;

    StopWriting(ioeltout_pos_cov);
    StopWriting(ioeltout_pos_cov_type);
    StopWriting(ioeltout_pos_lla);
    StopWriting(ioeltout_status);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}


void MAPSros_topic_subscriber::ROSTwistReceivedCallback(const geometry_msgs::Twist::ConstPtr& twist)
{
    try 
    {
    MAPSIOElt* ioeltout = StartWriting(Output(0));
	ioeltout->Float64(0) = twist->linear.x;
	ioeltout->Float64(1) = twist->linear.y;
	ioeltout->Float64(2) = twist->linear.z;
	ioeltout->Float64(3) = twist->angular.x;
	ioeltout->Float64(4) = twist->angular.y;
	ioeltout->Float64(5) = twist->angular.z;
	StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSTwistStampedReceivedCallback(const geometry_msgs::TwistStamped::ConstPtr& twist)
{
    try 
    {
    MAPSTimestamp t;
    if (false == m_transfer_ros_timestamp)
        t = MAPS::CurrentTime();
    else
        t = MAPSRosUtils::ROSTimeToMAPSTimestamp(twist->header.stamp);

    MAPSIOElt* ioeltout = StartWriting(Output(0));
    ioeltout->Float64(0) = twist->twist.linear.x;
    ioeltout->Float64(1) = twist->twist.linear.y;
    ioeltout->Float64(2) = twist->twist.linear.z;
    ioeltout->Float64(3) = twist->twist.angular.x;
    ioeltout->Float64(4) = twist->twist.angular.y;
    ioeltout->Float64(5) = twist->twist.angular.z;
    ioeltout->Timestamp() = t;
    StopWriting(ioeltout);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSOdometryReceivedCallback(const nav_msgs::Odometry::ConstPtr &odometry)
{
    try 
    {
    MAPSTimestamp t;
    if (false == m_transfer_ros_timestamp)
        t = MAPS::CurrentTime();
    else
        t = MAPSRosUtils::ROSTimeToMAPSTimestamp(odometry->header.stamp);

    MAPSIOElt* ioeltout_point = StartWriting(Output(0));
    MAPSIOElt* ioeltout_quat = StartWriting(Output(1));
    MAPSIOElt* ioeltout_pose_cov = StartWriting(Output(2));
    MAPSIOElt* ioeltout_twist = StartWriting(Output(3));
    MAPSIOElt* ioeltout_twist_cov = StartWriting(Output(4));
    ioeltout_point->Float64(0) = odometry->pose.pose.position.x;
    ioeltout_point->Float64(1) = odometry->pose.pose.position.y;
    ioeltout_point->Float64(2) = odometry->pose.pose.position.z;
    ioeltout_quat->Float64(0) = odometry->pose.pose.orientation.x;
    ioeltout_quat->Float64(1) = odometry->pose.pose.orientation.y;
    ioeltout_quat->Float64(2) = odometry->pose.pose.orientation.z;
    ioeltout_quat->Float64(3) = odometry->pose.pose.orientation.w;
    MAPSFloat64* pos_cov_out = &ioeltout_pose_cov->Float64();
    for (int i=0; i<36; i++)  
    {
        *(pos_cov_out++) = odometry->pose.covariance.at(i);
    }
    ioeltout_twist->Float64(0) = odometry->twist.twist.linear.x;
    ioeltout_twist->Float64(1) = odometry->twist.twist.linear.y;
    ioeltout_twist->Float64(2) = odometry->twist.twist.linear.z;
    ioeltout_twist->Float64(3) = odometry->twist.twist.angular.x;
    ioeltout_twist->Float64(4) = odometry->twist.twist.angular.y;
    ioeltout_twist->Float64(5) = odometry->twist.twist.angular.z;
    MAPSFloat64* twist_cov_out = &ioeltout_twist_cov->Float64();
    for (int i=0; i<36; i++) 
    {
        *(twist_cov_out++) = odometry->twist.covariance.at(i);
    }
    ioeltout_point->Timestamp()=t;
    ioeltout_quat->Timestamp()=t;
    ioeltout_pose_cov->Timestamp()=t;
    ioeltout_twist->Timestamp()=t;
    ioeltout_twist_cov->Timestamp()=t;
    StopWriting(ioeltout_point);
    StopWriting(ioeltout_quat);
    StopWriting(ioeltout_pose_cov);
    StopWriting(ioeltout_twist);
    StopWriting(ioeltout_twist_cov);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::ROSVisuMarkerReceivedCallback(const visualization_msgs::Marker::ConstPtr& callback_marker)
{
    try 
    {
        // write timestamp
        MAPSTimestamp marker_ts;
        if (false == m_transfer_ros_timestamp)
            marker_ts = MAPS::CurrentTime();
        else
            marker_ts = MAPSRosUtils::ROSTimeToMAPSTimestamp(callback_marker->header.stamp);

        if (m_first_time) 
        {
            m_first_time = false;

            Output(MARKER_OUTPUT_NB_ARROW).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_ARROW);
            Output(MARKER_OUTPUT_NB_CUBE).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_CUBE);
            Output(MARKER_OUTPUT_NB_SPHERE).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_SPHERE);
            Output(MARKER_OUTPUT_NB_CYLINDER).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_CYLINDER);
            Output(MARKER_OUTPUT_NB_LINE_STRIP).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_LINE_STRIP);
            Output(MARKER_OUTPUT_NB_LINE_LIST).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_LINE_LIST);
            Output(MARKER_OUTPUT_NB_CUBE_LIST).AllocOutputBuffer(1);
            Output(MARKER_OUTPUT_NB_SPHERE_LIST).AllocOutputBuffer(1);
            Output(MARKER_OUTPUT_NB_POINTS).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_POINTS);
            Output(MARKER_OUTPUT_NB_TEXT_VIEW_FACING).AllocOutputBuffer(1);
            Output(MARKER_OUTPUT_NB_MESH_RESOURCE).AllocOutputBuffer(1);
            Output(MARKER_OUTPUT_NB_TRIANGLE_LIST).AllocOutputBuffer(MARKER_OUTPUT_SIZE_MAX_TRIANGLE_LIST);
        }
        int concerned_output = m_markers_cache.apply_action(callback_marker);
        auto v = m_markers_cache.get_markers_by_output(concerned_output);

        //for each type run the vector, then StartWriting it in one time
        MarkersWriteByType(v, callback_marker->type, concerned_output, marker_ts);
    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::MarkersWriteByType(std::vector< visualization_msgs::Marker::ConstPtr >& markers, int32_t marker_type, int concerned_output, MAPSTimestamp marker_ts) {
    MAPSIOElt *outElt = StartWriting(Output(concerned_output));

    switch (marker_type) 
    {
        case visualization_msgs::Marker::ARROW :
        case visualization_msgs::Marker::CUBE :
        case visualization_msgs::Marker::SPHERE :
        case visualization_msgs::Marker::CYLINDER : 
        {
            // TODO: check buffersize as for POINTS?
            outElt->VectorSize() = MAPSInt32(markers.size());
            int i = 0;
            for (auto &marker : markers) 
            {
                MAPSRealObject &obj = outElt->RealObject(i);
                OutputMarkerOnlyFill(obj, marker);
                i++;
            }
            break;
        }
        case visualization_msgs::Marker::POINTS : 
        {
            int vector_size = 0; // first set output VectorSize
            for (auto& m : markers)
                vector_size += m->points.size();

            if (outElt->BufferSize() < vector_size) 
            {
                StopWriting(outElt);
                MAPSStreamedString ss;
                ss << "Too many points in marker: " << vector_size
                   << " current max:" << outElt->BufferSize();
                ss << " => try increasing it at output allocation.";
                Error(ss);
            }
            outElt->VectorSize() = vector_size;
            int i = 0;
            for (auto& marker : markers) 
            {
                for (auto p : marker->points) 
                {
                    MAPSRealObject &obj = outElt->RealObject(i);
                    OutputMarkerOnlyFill(obj, marker, i);
                    obj.x = p.x;
                    obj.y = p.y;
                    obj.z = p.z;
                    obj.custom.length = marker->scale.y;
                    i++;
                }
            }
            break;
        }
        case visualization_msgs::Marker::LINE_STRIP : 
        {
            int vector_size = 0; // first set output VectorSize
            for (auto& m : markers)
                vector_size += m->points.size() * 3;

            if (outElt->BufferSize() < vector_size) 
            {
                StopWriting(outElt);
                MAPSStreamedString ss;
                ss << "Too many points in line strip marker: " << vector_size
                   << " current max:" << outElt->BufferSize();
                ss << " => try increasing it at output allocation.";
                ss << " markers size: " << markers.size();
                Error(ss);
            }
            outElt->VectorSize() = vector_size;
            int i = 0;
            for (auto& marker : markers) 
            {
                for (auto p : marker->points) 
                {
                    outElt->Float64(i + 0) = p.x;
                    outElt->Float64(i + 1) = p.y;
                    outElt->Float64(i + 2) = p.z;
                    i += 3;
                }
            }
            break;
        }
        case visualization_msgs::Marker::LINE_LIST : 
        {
            int vector_size = 0; // first set output VectorSize
            for (auto& m : markers)
                vector_size += m->points.size() * 3;

            if (outElt->BufferSize() < vector_size) 
            {
                StopWriting(outElt);
                MAPSStreamedString ss;
                ss << "Too many points in line list marker: " << vector_size
                   << " current max:" << outElt->BufferSize();
                ss << " => try increasing it at output allocation.";
                ss << " markers size: " << markers.size();
                Error(ss);
            }
            outElt->VectorSize() = vector_size;
            int i = 0;
            for (auto& marker : markers) 
            {
                for (auto p : marker->points) 
                {
                    outElt->Float64(i + 0) = p.x;
                    outElt->Float64(i + 1) = p.y;
                    outElt->Float64(i + 2) = p.z;
                    i += 3;
                }
            }
            break;
        }
        case visualization_msgs::Marker::CUBE_LIST :
            break;
        case visualization_msgs::Marker::SPHERE_LIST :
            break;
        case visualization_msgs::Marker::TEXT_VIEW_FACING :
            break;
        case visualization_msgs::Marker::MESH_RESOURCE :
            break;
        case visualization_msgs::Marker::TRIANGLE_LIST : 
        {
            int vector_size = 0; // first set output VectorSize
            for (auto &m : markers)
                vector_size += m->points.size() * 3;
            if (outElt->BufferSize() < vector_size) 
            {
                StopWriting(outElt);
                MAPSStreamedString ss;
                ss << "Too many points in triangle list marker: " << vector_size
                   << " current max:" << outElt->BufferSize();
                ss << " => try increasing it at output allocation.";
                Error(ss);
            }
            outElt->VectorSize() = vector_size;
            int i = 0;
            for (auto &marker : markers) 
            {
                for (auto p : marker->points) 
                {
                    MAPSRealObject &obj = outElt->RealObject(i);
                    OutputMarkerOnlyFill(obj, marker, i);
                    obj.x = p.x;
                    obj.y = p.y;
                    obj.z = p.z;
                    obj.custom.length = marker->scale.y;
                    i++;
                }
            }
            break;
        }
        default: 
        {
            MAPSStreamedString ss;
            ss << "ROS Marker type unknown: " << marker_type;
            ReportWarning(ss);
            break;
        }
    }
    outElt->Timestamp() = marker_ts;
    StopWriting(outElt);
}

void MAPSros_topic_subscriber::ROSCANFrameReceivedCallback(const can_msgs::Frame::ConstPtr& frame)
{
    try 
    {
    MAPSTimestamp t;
    if (false == m_transfer_ros_timestamp)
        t = MAPS::CurrentTime();
    else
        t = MAPSRosUtils::ROSTimeToMAPSTimestamp(frame->header.stamp);

    MAPSIOElt* ioeltout = StartWriting(Output(0));
    MAPSCANFrame& frame_out = ioeltout->CANFrame();
    frame_out.arbitrationId = (frame->id & MAPSCANFrame::AddressMask);
    if (frame->is_extended)
        frame_out.arbitrationId |= MAPSCANFrame::ExtendedId;
    frame_out.isRemote = frame->is_rtr;
    frame_out.dataLength = frame->dlc;
    frame_out.ts = t;
    *(MAPSUInt64*)frame_out.data = *(MAPSUInt64*)&frame->data[0];
    ioeltout->Timestamp()=t;
    StopWriting(ioeltout);

    } 
    catch (int error) 
    {
        if (error == MAPS::ModuleDied)
            return;
        else
        {
            MAPSStreamedString ss;
            ss << "Error " << error << " caught in CAN frame reception callback. Stopping component.";
            ReportError(ss);
        }
    }
}

void MAPSros_topic_subscriber::OutputMarkerOnlyFill(MAPSRealObject& obj, const visualization_msgs::Marker::ConstPtr& marker, int)
{
    obj.kind = MAPSRealObject::Custom;
    obj.id = marker->id; // id
    obj.x = marker->pose.position.x;
    obj.y = marker->pose.position.y;
    obj.z = marker->pose.position.z;

    obj.color = MAPS_RGB(MAPSInt64(floor(marker->color.r*255)), MAPSInt64(floor(marker->color.g*255)), MAPSInt64(floor(marker->color.b*255)));
    obj.misc1 = MAPSInt32(0);
    obj.misc2 = MAPSInt32(0);
    obj.misc3 = MAPSInt32(0);

    MAPSCustomRealObjectKind &custom_obj = obj.custom;
    custom_obj.userKind = -1; //MAPSInt32(marker->type);
    custom_obj.width = marker->scale.x;
    custom_obj.height = marker->scale.y;
    custom_obj.length = marker->scale.z;
    custom_obj.phiAngle = marker->pose.orientation.x;
    custom_obj.thetaAngle = marker->pose.orientation.y;
    custom_obj.psiAngle = marker->pose.orientation.z;

}

void MAPSros_topic_subscriber::ROSVisuMarkerArrayReceivedCallback(const visualization_msgs::MarkerArray::ConstPtr& markers)
{
    for (auto m : markers->markers) 
    {
        auto p = boost::make_shared< const visualization_msgs::Marker >(m);
        ROSVisuMarkerReceivedCallback(p);
    }
}


//void MAPSros_topic_subscriber::RROSRmpAudioCommandReceivedCallback(const rmp_msgs::AudioCommand::ConstPtr& audio_command)
//{
//    try
//    {
//        MAPSTimestamp t;
//        if (false == m_transfer_ros_timestamp)
//            t = MAPS::CurrentTime();
//        else
//            t = MAPSRosUtils::ROSTimeToMAPSTimestamp(audio_command->header.stamp);
//
//        MAPSIOElt* ioeltout_command = StartWriting(Output(0));
//
//        ioeltout_command ->UnsignedInteger32() = audio_command->command;
//        ioeltout_command->Timestamp()=t;
//        StopWriting(ioeltout_command);
//    }
//    catch (int error)
//    {
//        if (error == MAPS::ModuleDied)
//            return;
//        throw error;
//    }
//}
//void MAPSros_topic_subscriber::RROSRmpBoolStampedReceivedCallback(const rmp_msgs::BoolStamped::ConstPtr& bool_stamped)
//{
//    try
//    {
//        MAPSTimestamp t;
//        if (false == m_transfer_ros_timestamp)
//            t = MAPS::CurrentTime();
//        else
//            t = MAPSRosUtils::ROSTimeToMAPSTimestamp(bool_stamped->header.stamp);
//
//        MAPSIOElt* ioeltout_data = StartWriting(Output(0));
//
//        ioeltout_data->UnsignedInteger8() = bool_stamped->data;
//
//        ioeltout_data->Timestamp()=t;
//
//        StopWriting(ioeltout_data);
//    }
//    catch (int error)
//    {
//        if (error == MAPS::ModuleDied)
//            return;
//        throw error;
//    }
//}
void MAPSros_topic_subscriber::ROSRmpBatteryReceivedCallback(const rmp_msgs::Battery::ConstPtr& battery)
{
    try
    {
        MAPSTimestamp t;
        if (false == m_transfer_ros_timestamp)
            t = MAPS::CurrentTime();
        else
            t = MAPSRosUtils::ROSTimeToMAPSTimestamp(battery->header.stamp);

        MAPSIOElt* ioeltout_charge_state = StartWriting(Output(0));
        MAPSIOElt* ioeltout_temperature = StartWriting(Output(1));
        MAPSIOElt* ioeltout_min_propulsion_charge_state = StartWriting(Output(2));
        MAPSIOElt* ioeltout_aux_battery_voltage = StartWriting(Output(3));
        MAPSIOElt* ioeltout_aux_battery_current = StartWriting(Output(4));
        MAPSIOElt* ioeltout_aux_abb_system_status = StartWriting(Output(5));
        MAPSIOElt* ioeltout_aux_battery_status = StartWriting(Output(6));

        for (int i=0; i<5; i++) {
            ioeltout_charge_state->Float32(i) = battery->charge_state[i];
        }

        for (int i=0; i<5; i++) {
            ioeltout_temperature->Float32(i) = battery->temperature[i];
        }

        ioeltout_min_propulsion_charge_state->Float32() = battery->min_propulsion_charge_state;
        ioeltout_aux_battery_voltage->Float32() = battery->aux_battery_voltage;
        ioeltout_aux_battery_current->Float32() = battery->aux_battery_current;
        ioeltout_aux_abb_system_status->Integer32() = battery->abb_system_status;
        ioeltout_aux_battery_status->Integer32() = battery->aux_battery_status;

        ioeltout_charge_state->Timestamp()=t;
        ioeltout_temperature->Timestamp()=t;
        ioeltout_min_propulsion_charge_state->Timestamp()=t;
        ioeltout_aux_battery_voltage->Timestamp()=t;
        ioeltout_aux_battery_current->Timestamp()=t;
        ioeltout_aux_abb_system_status->Timestamp()=t;
        ioeltout_aux_battery_status->Timestamp()=t;

        StopWriting(ioeltout_charge_state);
        StopWriting(ioeltout_temperature);
        StopWriting(ioeltout_min_propulsion_charge_state);
        StopWriting(ioeltout_aux_battery_voltage);
        StopWriting(ioeltout_aux_battery_current);
        StopWriting(ioeltout_aux_abb_system_status);
        StopWriting(ioeltout_aux_battery_status);
    }
    catch (int error)
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}
void MAPSros_topic_subscriber::ROSRmpFaultStatusReceivedCallback(const rmp_msgs::FaultStatus::ConstPtr& fault_status)
{
    try
    {
        MAPSTimestamp t;
        if (false == m_transfer_ros_timestamp)
            t = MAPS::CurrentTime();
        else
            t = MAPSRosUtils::ROSTimeToMAPSTimestamp(fault_status->header.stamp);

        MAPSIOElt* ioeltout_fault_list = StartWriting(Output(0));


        int vectorsize_out = fault_status->fault_list.size();


        char* data_out = ioeltout_fault_list->TextAscii();
        int txt_len_total=0;
        for (int i=0; i<vectorsize_out;i++)
        {
            int txt_len = fault_status->fault_list[i].size();

            MAPS::Memcpy(data_out,fault_status->fault_list[i].c_str(),txt_len);
            txt_len_total+=txt_len;

        }
        ioeltout_fault_list->TextAscii()[txt_len_total] = '\0';

        ioeltout_fault_list->VectorSize() = txt_len_total;
        ioeltout_fault_list->Timestamp() = t;
        StopWriting(ioeltout_fault_list);
    }
    catch (int error)
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}
void MAPSros_topic_subscriber::ROSRmpMotorStatusReceivedCallback(const rmp_msgs::MotorStatus::ConstPtr& motor_status)
{
    try
    {
        MAPSTimestamp t;
        if (false == m_transfer_ros_timestamp)
            t = MAPS::CurrentTime();
        else
            t = MAPSRosUtils::ROSTimeToMAPSTimestamp(motor_status->header.stamp);

        MAPSIOElt* ioeltout_current = StartWriting(Output(0));
        MAPSIOElt* ioeltout_current_limit = StartWriting(Output(1));
        MAPSIOElt* ioeltout_mcu_inst_power = StartWriting(Output(2));
        MAPSIOElt* ioeltout_mcu_total_energy = StartWriting(Output(3));
        MAPSIOElt* ioeltout_max_current = StartWriting(Output(4));
        MAPSIOElt* ioeltout_min_current_limit = StartWriting(Output(5));

        for (int i=0; i<4; i++) {
            ioeltout_current->Float32(i) = motor_status->current[i];
        }

        for (int i=0; i<4; i++) {
            ioeltout_current_limit->Float32(i) = motor_status->current_limit[i];
        }

        for (int i=0; i<4; i++) {
            ioeltout_mcu_inst_power->Float32(i) = motor_status->mcu_inst_power[i];
        }

        for (int i=0; i<4; i++) {
            ioeltout_mcu_total_energy->Float32(i) = motor_status->mcu_total_energy[i];
        }

        ioeltout_max_current->Float32() = motor_status->max_current;
        ioeltout_min_current_limit->Float32() = motor_status->min_current_limit;


        ioeltout_current->Timestamp()=t;
        ioeltout_current_limit->Timestamp()=t;
        ioeltout_mcu_inst_power->Timestamp()=t;
        ioeltout_mcu_total_energy->Timestamp()=t;
        ioeltout_max_current->Timestamp()=t;
        ioeltout_min_current_limit->Timestamp()=t;


        StopWriting(ioeltout_current);
        StopWriting(ioeltout_current_limit);
        StopWriting(ioeltout_mcu_inst_power);
        StopWriting(ioeltout_mcu_total_energy);
        StopWriting(ioeltout_max_current);
        StopWriting(ioeltout_min_current_limit);
    }
    catch (int error)
    {
        if (error == MAPS::ModuleDied)
            return;
        throw error;
    }
}

void MAPSros_topic_subscriber::Core()
{
    Wait4Event(isDyingEvent);
}

void MAPSros_topic_subscriber::Death()
{
    if (m_sub) 
    {
        m_sub->shutdown();
        MAPS_SAFE_DELETE(m_sub);
    }
    if (m_n) 
    {
         m_n = nullptr; //The ROS node will be killed/released by the RTMaps ROS Bridge CoreFunction.
    }
    m_markers_cache.clear();
}
