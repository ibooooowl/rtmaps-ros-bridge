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

#include "maps_ros_topic_publisher.h"	// Includes the header of this component

// Use the macros to declare the inputs
MAPS_BEGIN_INPUTS_DEFINITION(MAPSros_topic_publisher)
    MAPS_INPUT("input_int32",MAPS::FilterInteger32,MAPS::FifoReader)
    MAPS_INPUT("input_int64",MAPS::FilterInteger64,MAPS::FifoReader)
    MAPS_INPUT("input_float32",MAPS::FilterFloat32,MAPS::FifoReader)
    MAPS_INPUT("input_float64",MAPS::FilterFloat64,MAPS::FifoReader)
    MAPS_INPUT("input_text",MAPS::FilterTextAscii,MAPS::FifoReader)
    MAPS_INPUT("input_image",MAPS::FilterIplImage,MAPS::FifoReader)
    MAPS_INPUT("input_mapsimage",MAPS::FilterMAPSImage,MAPS::FifoReader)
    MAPS_INPUT("input_pointcloud_xyz",MAPS::FilterNumbers,MAPS::FifoReader)
    MAPS_INPUT("input_uint8",MAPS::FilterUnsignedInteger8,MAPS::FifoReader)
    MAPS_INPUT("input_uint32",MAPS::FilterUnsignedInteger32,MAPS::FifoReader)
MAPS_END_INPUTS_DEFINITION

// Use the macros to declare the outputs
MAPS_BEGIN_OUTPUTS_DEFINITION(MAPSros_topic_publisher)
    //MAPS_OUTPUT("oName",MAPS::Integer,NULL,NULL,1)
MAPS_END_OUTPUTS_DEFINITION

// Use the macros to declare the properties
MAPS_BEGIN_PROPERTIES_DEFINITION(MAPSros_topic_publisher)
    MAPS_PROPERTY("topic_name","rtmaps/ros_topic_name",false,false)
    MAPS_PROPERTY_ENUM("topic_type","None",0,false,false)
    MAPS_PROPERTY("frame_id","map",false,false)
    MAPS_PROPERTY_ENUM("message","None",0,false,false) //Text|Image|Int 32|Int 32 array|Int 64|Int 64 array|Float 32|Float 32 array|Float 64|Float 64 array|Laser scan|Twist",0,false,false)
    MAPS_PROPERTY_ENUM("published_timestamps","RTMaps samples timestamps|ROS current time",1,false,false)
    MAPS_PROPERTY("laser_min_angle",-135.0,false,false)
    MAPS_PROPERTY("laser_max_angle",135.0,false,false)
    MAPS_PROPERTY("laser_angle_increment",1.0,false,false)
    MAPS_PROPERTY("laser_time_increment",0.0,false,false)
    MAPS_PROPERTY("laser_scan_time",0.0,false,false)
    MAPS_PROPERTY("laser_min_range",0.0,false,false)
    MAPS_PROPERTY("laser_max_range",100.0,false,false)
    MAPS_PROPERTY("laser_supports_intensities",false,false,false)
    //MAPS_PROPERTY_ENUM("pointcloud2_input_type","Distances|XYZ",0,false,false)
    MAPS_PROPERTY("pointcloud2_width",-1,false,false)
    MAPS_PROPERTY("pointcloud2_height",1,false,false)
    MAPS_PROPERTY("pointcloud2_is_dense",true,false,false)
    MAPS_PROPERTY_ENUM("pointcloud2_datatype","Auto|Integer32|Float32|Double (Float64)",0,false,false)
    MAPS_PROPERTY("odom_child_frame_id",(const char*)NULL,false,false)
MAPS_END_PROPERTIES_DEFINITION

// Use the macros to declare the actions
MAPS_BEGIN_ACTIONS_DEFINITION(MAPSros_topic_publisher)
    //MAPS_ACTION("aName",MAPSros_topic_publisher::ActionName)
MAPS_END_ACTIONS_DEFINITION

//Vesrion 1.1: fixed encoding of images "rgba8" and "bgra8" instead of "rgba" and "bgra"
//Version 1.2: rewritten way to specify topic type and message type.
//Version 1.3: added support for PointCloud2
//Version 1.4: fixed potential issue with PointCloud2 data field allocation on uninitialized variable.
//Version 1.5: ???
//Version 1.6: added odometry support (nav_msgs)
//Version 1.7: added header in odometry msgs.
//Version 1.8: support for variable size PointCloud2.
//Version 1.9: added child_frame_id for Odometry msgs (nav).
//Version 2.0: added NavSatFix and IMU messages
//Version 2.1: changed way ROS node are managed (using singleton). No more error message that CompressedImage is not supported by default. Fixed data type issue for float64 publishing PointCloud2.
//Version 2.2: added support for TwistStamped msg.
//Version 2.2.1: corrected ROS time to RTMaps time conversion.
//Version 2.2.2: fixed float64 scalars publishing.

// Use the macros to declare this component (ros_topic_publisher) behaviour
MAPS_COMPONENT_DEFINITION(MAPSros_topic_publisher,"ros_topic_publisher","2.2.2",128,
			  MAPS::Threaded,MAPS::Threaded,
			  0, // Nb of inputs. Leave -1 to use the number of declared input definitions
			  -1, // Nb of outputs. Leave -1 to use the number of declared output definitions
              4, // Nb of properties. Leave -1 to use the number of declared property definitions
			  -1) // Nb of actions. Leave -1 to use the number of declared action definitions


MAPSros_topic_publisher::MAPSros_topic_publisher(const char* name, MAPSComponentDefinition& cd) :
MAPSComponent(name,cd) 
{
	MAPSEnumStruct topic_types;
	for (unsigned int i=0; i< sizeof(s_topic_types)/sizeof(const char*); i++) 
	{
		topic_types.enumValues->Append() = s_topic_types[i];
	}
	DirectSet(Property("topic_type"),topic_types);
}

void MAPSros_topic_publisher::Dynamic()
{
	m_topic_type = (int)GetIntegerProperty("topic_type");
	int selected_message = (int)GetIntegerProperty("message");
	if (Property("topic_type").PropertyChanged())
	 {
		Property("topic_type").AcknowledgePropertyChanged();

        selected_message = 0;


	}
	MAPSEnumStruct messages;

	switch (m_topic_type) 
	{
	case TOPIC_TYPE_STD:
		for (unsigned int i=0; i < sizeof(s_std_msgs)/sizeof(const char*); i++) 
		{
			messages.enumValues->Append() = s_std_msgs[i];
		}
		break;
	case TOPIC_TYPE_SENSOR:
		for (unsigned int i=0; i < sizeof(s_sensor_msgs)/sizeof(const char*); i++) 
		{
			messages.enumValues->Append() = s_sensor_msgs[i];
		}
		break;
	case TOPIC_TYPE_GEOM:
		for (unsigned int i=0; i < sizeof(s_geometry_msgs)/sizeof(const char*); i++) 
		{
			messages.enumValues->Append() = s_geometry_msgs[i];
		}
		break;
    case TOPIC_TYPE_NAV:
        for (unsigned int i=0; i < sizeof(s_nav_msgs)/sizeof(const char*); i++) 
		{
            messages.enumValues->Append() = s_nav_msgs[i];
        }
        break;
    case TOPIC_TYPE_RMP:
        for (unsigned int i=3; i < sizeof(s_rmp_msgs)/sizeof(const char*); i++)
        {
            messages.enumValues->Append() = s_rmp_msgs[i];
        }
        break;
	default :
		messages.enumValues->Append() = "None";
		ReportError("This topic type is not supported yet..");
		break;
	}
	if (selected_message >= messages.enumValues->Size())
		selected_message = 0;
	messages.selectedEnum = selected_message;
	DirectSet(Property("message"),messages);
	m_message = selected_message;

	m_output_header = false;
	switch(m_topic_type) {
	case TOPIC_TYPE_STD:
		m_nb_inputs = CreateIOsForStdTopics(&m_output_header);
		break;
	case TOPIC_TYPE_SENSOR:
		m_nb_inputs = CreateIOsForSensorTopics(&m_output_header);
		break;
	case TOPIC_TYPE_GEOM:
		m_nb_inputs = CreateIOsForGeomTopics(&m_output_header);
		break;
    case TOPIC_TYPE_NAV:
        m_nb_inputs = CreateIOsForNavTopics(&m_output_header);
        break;
    case TOPIC_TYPE_RMP:
        m_nb_inputs = CreateIOsForRmpTopics(&m_output_header);
        break;
	}

	if (m_output_header) 
	{
		NewProperty("published_timestamps");
		int pub_ts = (int)GetIntegerProperty("published_timestamps");
		if (pub_ts == 0)
			m_publish_rtmaps_timestamp = true;
		else
			m_publish_rtmaps_timestamp = false;
	}
}

int MAPSros_topic_publisher::CreateIOsForStdTopics(bool* output_header)
{
	switch(m_message) 
	{
	case STD_MSG_INT32:
		NewInput("input_int32");
		break;
	case STD_MSG_INT32_ARRAY:
		NewInput("input_int32","input_int32_array");
		break;
	case STD_MSG_INT64:
		NewInput("input_int64");
		break;
	case STD_MSG_INT64_ARRAY:
		NewInput("input_int64","input_int64_array");
		break;
	case STD_MSG_FLOAT32:
		NewInput("input_float32");
		break;
	case STD_MSG_FLOAT32_ARRAY:
		NewInput("input_float32","input_float32_array");
		break;
	case STD_MSG_FLOAT64:
		NewInput("input_float64");
		break;
	case STD_MSG_FLOAT64_ARRAY:
		NewInput("input_float64","input_float64_array");
		break;
	case STD_MSG_TEXT:
		NewInput("input_text");
		break;
	}
	if (output_header)
		*output_header = false;
	return 1; //created 1 input.
}

int MAPSros_topic_publisher::CreateIOsForSensorTopics(bool* output_header)
{
	if (output_header)
		*output_header = false;
	int nb_inputs = 1;
	switch(m_message) 
	{
	case SENSOR_MSG_COMPRESSED_IMAGE:
		NewInput("input_mapsimage","compressed_image");
		if (output_header)
			*output_header = true;
		break;
	case SENSOR_MSG_IMAGE:
		NewInput("input_image");
		if (output_header)
			*output_header = true;
		break;
	case SENSOR_MSG_LASER_SCAN:
		if (output_header)
			*output_header = true;
	    NewProperty("laser_min_angle");
		NewProperty("laser_max_angle");
		NewProperty("laser_angle_increment");
		NewProperty("laser_time_increment");
		NewProperty("laser_scan_time");
		NewProperty("laser_min_range");
		NewProperty("laser_max_range");
		NewProperty("laser_supports_intensities");

		m_laser_supports_intens = GetBoolProperty("laser_supports_intensities");
		NewInput("input_float64","input_laser_scan_ranges");
		if (m_laser_supports_intens) 
		{
			nb_inputs = 2;
			NewInput("input_float64","input_laser_scan_intensities");
		}
		break;
    case SENSOR_MSG_POINT_CLOUD2:
        if (output_header)
            *output_header = true;
        //NewProperty("pointcloud2_input_type");
        /*if (GetIntegerProperty("pointcloud2_input_type") < 2) {
            NewProperty("pointcloud2_width");
            NewProperty("pointcloud2_height");
        }*/
        NewProperty("pointcloud2_width");
        NewProperty("pointcloud2_height");
        NewProperty("pointcloud2_is_dense");
        NewProperty("pointcloud2_datatype");
        NewInput("input_pointcloud_xyz");
        break;
    case SENSOR_MSG_NAV_SAT_FIX:
        if (output_header)
            *output_header = true;
        NewInput("input_int32","navsatfix_status");
        NewInput("input_float64","navsatifx_pos_lla");
        NewInput("input_float64","navsatfix_pos_cov");
        NewInput("input_int32","navsatfix_cov_type");
        nb_inputs = 4;
        break;
    case SENSOR_MSG_IMU:
        if (output_header)
            *output_header = true;
        NewInput("input_float64","orientation_quaternion");
        NewInput("input_float64","angular_velocities");
        NewInput("input_float64","accelerations");
        nb_inputs = 3;
        break;
	default:
		ReportError("This topic is not supported yet.");
		nb_inputs = 0;
	}
	return nb_inputs;
}

int MAPSros_topic_publisher::CreateIOsForGeomTopics(bool* output_header)
{
	if (output_header)
		*output_header = false;
	int nb_inputs = 1;
	switch(m_message) 
	{
		case GEOM_MSG_POINT:
			NewInput("input_float64","input_point");
		break;
		case GEOM_MSG_TWIST:
			NewInput("input_float64","input_twist");
			break;
    case GEOM_MSG_TWIST_STAMPED:
            NewInput("input_float64","input_twist");
            if (output_header)
                *output_header = true;
            DirectSet(Property("topic_name"),"/rmp440le/base/vel_cmd");
            break;
		default:
			ReportError("This topic is not supported yet.");
	}

	return nb_inputs;
}

int MAPSros_topic_publisher::CreateIOsForNavTopics(bool* output_header)
{
    if (output_header)
        *output_header = false;
    int nb_inputs = 1;
    switch(m_message) 
	{
        case NAV_MSG_ODOMETRY:
        if (output_header)
            *output_header = true;
        NewInput("input_float64","input_pose_position");
        NewInput("input_float64","input_pose_orientation");
        NewInput("input_float64","input_pos_covariance");
        NewInput("input_float64","input_twist");
        NewInput("input_float64","input_twist_covariance");
        NewProperty("odom_child_frame_id");
        nb_inputs = 5;
        break;
    default:
        ReportError("This topic type is not supported yet.");
    }
    return nb_inputs;
}

int MAPSros_topic_publisher::CreateIOsForRmpTopics(bool* output_header)
{
    if (output_header)
        *output_header = false;
    int nb_inputs = 1;
    switch(m_message+3)
    {
        case RMP_MSG_BOOL_STAMPED:
            if (output_header)
                *output_header = true;
            NewInput("input_uint8","input_deadman");
            DirectSet(Property("topic_name"),"/rmp440le/deadman");
            break;
        case RMP_MSG_AUDIO_COMMAND:
            if (output_header)
                *output_header = true;
            NewInput("input_uint32","input_audio_command");
            DirectSet(Property("topic_name"),"/rmp440le/audio_cmd");
            break;
        default:
            ReportError("This topic type is not supported yet for IO creation.");
    }
    return nb_inputs;
}

void MAPSros_topic_publisher::Birth()
{
	m_first_time = true;
	m_n = NULL;
	m_pub = NULL;
    m_pointcloud2_channel = NULL;
	MAPSString topic_name = GetStringProperty("topic_name");
	try 
	{
		m_n = MAPSRosUtils::GetROSNode();
		if (m_n == NULL)
			Error("Could not create NodeHandle.");
		m_pub = new ros::Publisher();
		if (m_pub == NULL)
			Error("Could not create Publisher.");
	} 
	catch (ros::Exception const& e ) 
	{
		ReportError(e.what());
	}
	switch(m_topic_type) 
	{
	case TOPIC_TYPE_STD:
	{
		switch(m_message) 
		{
		case STD_MSG_INT32:
			*m_pub = m_n->advertise<std_msgs::Int32>((const char*)topic_name,100);
			break;
		case STD_MSG_INT32_ARRAY :
			*m_pub = m_n->advertise<std_msgs::Int32MultiArray>((const char*)topic_name,100);
			break;
		case STD_MSG_INT64 :
			*m_pub = m_n->advertise<std_msgs::Int64>((const char*)topic_name,100);
			break;
		case STD_MSG_INT64_ARRAY :
			*m_pub = m_n->advertise<std_msgs::Int64MultiArray>((const char*)topic_name,100);
			break;
		case STD_MSG_FLOAT32 :
			*m_pub = m_n->advertise<std_msgs::Float32>((const char*)topic_name,100);
			break;
		case STD_MSG_FLOAT32_ARRAY :
			*m_pub = m_n->advertise<std_msgs::Float32MultiArray>((const char*)topic_name,100);
			break;
		case STD_MSG_FLOAT64 :
			*m_pub = m_n->advertise<std_msgs::Float64>((const char*)topic_name,100);
			break;
		case STD_MSG_FLOAT64_ARRAY :
			*m_pub = m_n->advertise<std_msgs::Float64MultiArray>((const char*)topic_name,100);
			break;
		case STD_MSG_TEXT:
			*m_pub = m_n->advertise<std_msgs::String>((const char*)topic_name,1000);
			break;
		default:
			Error("Selected topic is not supported yet.");
		}
	}
		break;
	case TOPIC_TYPE_SENSOR:
		switch(m_message) 
		{
		case SENSOR_MSG_COMPRESSED_IMAGE:
			*m_pub = m_n->advertise<sensor_msgs::CompressedImage>((const char*)topic_name,10);
			break;
		case SENSOR_MSG_IMAGE:
			*m_pub = m_n->advertise<sensor_msgs::Image>((const char*)topic_name,10);
			break;
		case SENSOR_MSG_LASER_SCAN:
			*m_pub = m_n->advertise<sensor_msgs::LaserScan>((const char*)topic_name,16);
			break;
        case SENSOR_MSG_POINT_CLOUD2:
            *m_pub = m_n->advertise<sensor_msgs::PointCloud2>((const char*)topic_name,4);
            break;
        case SENSOR_MSG_NAV_SAT_FIX:
            *m_pub = m_n->advertise<sensor_msgs::NavSatFix>((const char*)topic_name,16);
            break;
        case SENSOR_MSG_IMU:
            *m_pub = m_n->advertise<sensor_msgs::Imu>((const char*)topic_name,16);
            break;
		default:
			Error("Selected topic is not supported yet.");
		}
		break;
	case TOPIC_TYPE_GEOM:
		switch(m_message) 
		{
		case GEOM_MSG_POINT:
			*m_pub = m_n->advertise<geometry_msgs::Point>((const char*)topic_name,100);
			break;
		case GEOM_MSG_TWIST:
			*m_pub = m_n->advertise<geometry_msgs::Twist>((const char*)topic_name,100);
			break;
        case GEOM_MSG_TWIST_STAMPED:
            *m_pub = m_n->advertise<geometry_msgs::TwistStamped>((const char*)topic_name,100);
            break;
		default:
			Error("Selected topic is not supported yet.");
		}
		break;
    case TOPIC_TYPE_NAV:
        switch(m_message) 
		{
        case NAV_MSG_ODOMETRY:
            *m_pub = m_n->advertise<nav_msgs::Odometry>((const char*)topic_name,64);
            break;
        default:
            Error("Selected topic is not supported yet.");
        }

        break;
    case TOPIC_TYPE_RMP:
        switch(m_message)
        {
            case RMP_MSG_BOOL_STAMPED:
                *m_pub = m_n->advertise<rmp_msgs::BoolStamped>((const char*)topic_name,100);
                break;
            case RMP_MSG_AUDIO_COMMAND:
                *m_pub = m_n->advertise<rmp_msgs::AudioCommand>((const char*)topic_name,100);
                break;
            default:
                Error("Selected topic is not supported yet.");
        }

        break;
	default:
		Error("Selected topic is not supported yet.");
	}

	m_count = 0;

    for (int i=0; i<m_nb_inputs; i++) 
	{
        m_inputs[i] = &Input(i);
	}
}

void MAPSros_topic_publisher::Core()
{
	MAPSTimestamp t;
	if (m_nb_inputs == 1) 
	{
		m_ioeltin = StartReading(Input(0));
		if (m_ioeltin == NULL)
			return;
		t = m_ioeltin->Timestamp();
	} 
	else 
	{
		t = SynchroStartReading(m_nb_inputs,m_inputs,m_ioelts);
		if (t < 0)
			return;
	}

	if (m_output_header) 
	{
        MAPSString frame_id = (const char*)GetStringProperty("frame_id");
        m_header.frame_id = frame_id.Len() > 0 ? (const char*)frame_id : (const char*)this->Name();
		if (m_publish_rtmaps_timestamp)
            m_header.stamp = MAPSRosUtils::MAPSTimestampToROSTime(t);
		else
			m_header.stamp = ros::Time::now();
	}


	switch(m_topic_type) 
	{
	case TOPIC_TYPE_STD:
		PublishStdMsg();
		break;
	case TOPIC_TYPE_SENSOR:
		PublishSensorMsg();
		break;
	case TOPIC_TYPE_GEOM:
		PublishGeomMsg();
		break;
    case TOPIC_TYPE_NAV:
        PublishNavMsg();
        break;
    case TOPIC_TYPE_RMP:
        PublishRmpMsg();
        break;
	}

	m_count++;
}

void MAPSros_topic_publisher::PublishStdMsg()
{
	switch(m_message) 
	{
	case STD_MSG_TEXT:
	{
		std_msgs::String msg;
		msg.data = (const char*)m_ioeltin->TextAscii();
		m_pub->publish(msg);
	}
		break;
	case STD_MSG_INT32:
	{
		std_msgs::Int32 msg;
		msg.data = m_ioeltin->Integer32();
		m_pub->publish(msg);
	}
		break;
	case STD_MSG_INT32_ARRAY:
	{
		std_msgs::Int32MultiArray msg;
		int vectorsize_in = m_ioeltin->VectorSize();

		msg.data.resize(vectorsize_in);
		for (int i=0; i<vectorsize_in; i++) 
		{
			msg.data[i] = m_ioeltin->Integer32(i);
		}

		msg.layout.data_offset=0;
		msg.layout.dim.resize(1);
		msg.layout.dim[0].label="vector";
		msg.layout.dim[0].size=vectorsize_in;
		msg.layout.dim[0].stride=vectorsize_in*sizeof(int32_t);

		m_pub->publish(msg);
	}
	break;
	case STD_MSG_INT64:
	{
		std_msgs::Int64 msg;
		msg.data = m_ioeltin->Integer64();
		m_pub->publish(msg);
	}
		break;
	case STD_MSG_INT64_ARRAY:
	{
		std_msgs::Int64MultiArray msg;
		int vectorsize_in = m_ioeltin->VectorSize();

		msg.data.resize(vectorsize_in);
		for (int i=0; i<vectorsize_in; i++) 
		{
			msg.data[i] = m_ioeltin->Integer64(i);
		}

		msg.layout.data_offset=0;
		msg.layout.dim.resize(1);
		msg.layout.dim[0].label="vector";
		msg.layout.dim[0].size=vectorsize_in;
		msg.layout.dim[0].stride=vectorsize_in*sizeof(int32_t);

		m_pub->publish(msg);
	}
	break;
	case STD_MSG_FLOAT32:
	{
		std_msgs::Float32 msg;
		msg.data = m_ioeltin->Float32();
		m_pub->publish(msg);
	}
		break;
	case STD_MSG_FLOAT32_ARRAY:
	{
		std_msgs::Float32MultiArray msg;
		int vectorsize_in = m_ioeltin->VectorSize();

		msg.data.resize(vectorsize_in);
		for (int i=0; i<vectorsize_in; i++) 
		{
			msg.data[i] = m_ioeltin->Float32(i);
		}

		msg.layout.data_offset=0;
		msg.layout.dim.resize(1);
		msg.layout.dim[0].label="vector";
		msg.layout.dim[0].size=vectorsize_in;
		msg.layout.dim[0].stride=vectorsize_in*sizeof(int32_t);

		m_pub->publish(msg);
	}
	break;
	case STD_MSG_FLOAT64:
	{
		std_msgs::Float64 msg;
		msg.data = m_ioeltin->Float64();
		m_pub->publish(msg);
	}
		break;
	case STD_MSG_FLOAT64_ARRAY:
	{
		std_msgs::Float64MultiArray msg;
		int vectorsize_in = m_ioeltin->VectorSize();

		msg.data.resize(vectorsize_in);
		for (int i=0; i<vectorsize_in; i++) {
			msg.data[i] = m_ioeltin->Float64(i);
		}

		msg.layout.data_offset=0;
		msg.layout.dim.resize(1);
		msg.layout.dim[0].label="vector";
		msg.layout.dim[0].size=vectorsize_in;
		msg.layout.dim[0].stride=vectorsize_in*sizeof(int32_t);

		m_pub->publish(msg);
	}
	break;
	}
}

void MAPSros_topic_publisher::PublishSensorMsg()
{
	switch(m_message) 
	{
		case SENSOR_MSG_COMPRESSED_IMAGE :
		{
			MAPSImage& image_in = m_ioeltin->MAPSImage();
			if (m_first_time) 
			{
				m_first_time = false;
				if(*(MAPSUInt32*)image_in.imageCoding != MAPS_IMAGECODING_JPEG &&
				   *(MAPSUInt32*)image_in.imageCoding != MAPS_IMAGECODING_PNG) {
						Error("Unsupported format for compressed image. ROS supports only jpeg and png images.");
					}
					switch(*(MAPSUInt32*)image_in.imageCoding) {
					case MAPS_IMAGECODING_JPEG:
						m_ros_comp_img.format = "jpeg";
						break;
					case MAPS_IMAGECODING_PNG:
						m_ros_comp_img.format = "png";
						break;
					default:
						Error("Unsupported image format.");
					}
			}
			m_ros_comp_img.data.resize(image_in.imageSize);
			m_ros_comp_img.header = m_header;
			MAPS::Memcpy((char*)&m_ros_comp_img.data[0],image_in.imageData,image_in.imageSize);
			m_pub->publish(m_ros_comp_img);
		}
		break;
		case SENSOR_MSG_IMAGE :
		{
			IplImage& image_in = m_ioeltin->IplImage();
			if (m_first_time) 
			{
				m_first_time = false;
				m_ros_img.height = image_in.height;
				m_ros_img.width = image_in.width;
				m_ros_img.is_bigendian = false;
				switch(*(MAPSUInt32*)image_in.channelSeq) 
				{
				case MAPS_CHANNELSEQ_BGR:
					m_ros_img.encoding = "bgr8";
					break;
				case MAPS_CHANNELSEQ_RGB:
					m_ros_img.encoding = "rgb8";
					break;
				case MAPS_CHANNELSEQ_GRAY:
					m_ros_img.encoding = "mono8";
					break;
				case MAPS_CHANNELSEQ_RGBA:
					m_ros_img.encoding = "rgba8";
					break;
				case MAPS_CHANNELSEQ_BGRA:
					m_ros_img.encoding = "bgra8";
					break;
				default:
					Error("Unsupported image format");
				}
				m_ros_img.step = image_in.widthStep;
				m_ros_img.data.resize(image_in.imageSize);
			}
			m_ros_img.header = m_header;
			MAPS::Memcpy((char*)&m_ros_img.data[0],image_in.imageData,image_in.imageSize);
			m_pub->publish(m_ros_img);
		}
			break;
		case SENSOR_MSG_LASER_SCAN:
		{
			m_ros_laser_scan.header = m_header;
			if (m_first_time) 
			{
				m_first_time = false;
				m_ros_laser_scan.angle_min = GetFloatProperty("laser_min_angle")*MAPS_PI/180.0;
				m_ros_laser_scan.angle_max = GetFloatProperty("laser_max_angle")*MAPS_PI/180.0;
				m_ros_laser_scan.angle_increment = GetFloatProperty("laser_angle_increment")*MAPS_PI/180.0;
				m_ros_laser_scan.time_increment = GetFloatProperty("laser_time_increment");
				m_ros_laser_scan.scan_time = GetFloatProperty("laser_scan_time");
				m_ros_laser_scan.range_min = GetFloatProperty("laser_min_range");
				m_ros_laser_scan.range_max = GetFloatProperty("laser_max_range");
				if (m_nb_inputs == 1) 
				{
					m_ros_laser_scan.ranges.resize(m_ioeltin->BufferSize());
				} 
				else 
				{
					m_ros_laser_scan.ranges.resize(m_ioelts[0]->BufferSize());
					m_ros_laser_scan.intensities.resize(m_ioelts[1]->BufferSize());
				}
			}

			if (m_nb_inputs == 1) 
			{
				int vectorsize_in = m_ioeltin->VectorSize();
				m_ros_laser_scan.ranges.resize(vectorsize_in);
				MAPSFloat64* data_in = &m_ioeltin->Float64();
				for (int i=0; i<vectorsize_in; i++) 
				{
					m_ros_laser_scan.ranges[i] = *(data_in++);
				}
			} 
			else 
			{
				int vectorsize_in = m_ioelts[0]->VectorSize();
				if (m_ioelts[1]->VectorSize() != vectorsize_in) 
				{
					ReportWarning("Different number of data samples on ranges and intensities vector. Discarding...");
					break;
				}
				m_ros_laser_scan.ranges.resize(vectorsize_in);
				m_ros_laser_scan.intensities.resize(vectorsize_in);
				MAPSFloat64* data_in_ranges = &m_ioelts[0]->Float64();
				MAPSFloat64* data_in_intens = &m_ioelts[1]->Float64();
				for (int i=0; i<vectorsize_in; i++) 
				{
					m_ros_laser_scan.ranges[i] = *(data_in_ranges++);
					m_ros_laser_scan.intensities[i] = *(data_in_intens++);
				}
			}

			m_pub->publish(m_ros_laser_scan);
		}
		break;
        case  SENSOR_MSG_POINT_CLOUD2:
        {
            m_ros_pointcloud2.header = m_header;
            if (m_first_time) 
			{
                m_first_time = false;
                m_ros_pointcloud2_output_type = (int)GetIntegerProperty("pointcloud2_datatype");
                m_ros_pointcloud2.is_dense = GetBoolProperty("pointcloud2_is_dense");
                m_ros_pointcloud2.is_bigendian = false;
                m_ros_pointcloud2.fields.resize(3);
                m_ros_pointcloud2.fields[0].name = "x";
                m_ros_pointcloud2.fields[1].name = "y";
                m_ros_pointcloud2.fields[2].name = "z";
                m_ros_pointcloud2_memcpy = false;
                m_ros_pointcloud2.fields[0].count = 1;
                m_ros_pointcloud2.fields[1].count = 1;
                m_ros_pointcloud2.fields[2].count = 1;

                if (MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterInteger64))
                    Error("64 bit integer point clouds are not supported in ROS.");

                if(m_ros_pointcloud2_output_type == 1 || (m_ros_pointcloud2_output_type == 0 && MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterInteger32))) 
				{
                    m_ros_pointcloud2.fields[0].offset = 0;
                    m_ros_pointcloud2.fields[1].offset = 4;
                    m_ros_pointcloud2.fields[2].offset = 8;
                    m_ros_pointcloud2.point_step = 4 * 3;
                    m_ros_pointcloud2.fields[0].datatype = 5; //Int32
                    m_ros_pointcloud2.fields[1].datatype = 5; //Int32
                    m_ros_pointcloud2.fields[2].datatype = 5; //Int32
                    if (m_ros_pointcloud2_output_type == 0 || (m_ros_pointcloud2_output_type == 1 && MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterInteger32)))
                        m_ros_pointcloud2_memcpy = true;
                    else 
					{
                        /*
                        if (MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterFloat32))
                            _pointcloud2_channel = new IOEltToPointCloud2<MAPSInt32, MAPSFloat32>();
                        else if (MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterFloat64))
                            _pointcloud2_channel = new IOEltToPointCloud2<MAPSInt32, MAPSFloat64>();
                            */
                    }
                } 
				else if (m_ros_pointcloud2_output_type == 2 || (m_ros_pointcloud2_output_type == 0 && MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterFloat32))) 
				{
                    m_ros_pointcloud2.point_step = 4 * 3;
                    m_ros_pointcloud2.fields[0].offset = 0;
                    m_ros_pointcloud2.fields[1].offset = 4;
                    m_ros_pointcloud2.fields[2].offset = 8;
                    m_ros_pointcloud2.fields[0].datatype = 7; // Float32
                    m_ros_pointcloud2.fields[1].datatype = 7; // Float32
                    m_ros_pointcloud2.fields[2].datatype = 7; // Float32
                    if (m_ros_pointcloud2_output_type == 0 || (m_ros_pointcloud2_output_type == 2 && MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterFloat32)))
                        m_ros_pointcloud2_memcpy = true;
                    else 
					{
                        /*
                        if (MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterInteger32))
                            _pointcloud2_channel = new IOEltToPointCloud2<MAPSFloat32, MAPSInt32>();
                        else if (MAPS::TypeFilter(m_ioeltin->Type(), MAPS::FilterFloat64))
                            _pointcloud2_channel = new IOEltToPointCloud2<MAPSFloat32,MAPSFloat64> ();
                            */
                    }

                } 
				else if (m_ros_pointcloud2_output_type == 3 || (m_ros_pointcloud2_output_type == 0 && MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterFloat64))) 
				{
                    m_ros_pointcloud2.point_step = 8 * 3;
                    m_ros_pointcloud2.fields[0].offset = 0;
                    m_ros_pointcloud2.fields[1].offset = 8;
                    m_ros_pointcloud2.fields[2].offset = 16;
                    m_ros_pointcloud2.fields[0].datatype = 8; // Float64
                    m_ros_pointcloud2.fields[1].datatype = 8; // Float64
                    m_ros_pointcloud2.fields[2].datatype = 8; // Float64
                    if (m_ros_pointcloud2_output_type == 0 || (m_ros_pointcloud2_output_type == 3 && MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterFloat64)))
                        m_ros_pointcloud2_memcpy = true;
                    else 
					{
                        /*
                        if (MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterInteger32))
                            _pointcloud2_channel = new IOEltToPointCloud2<MAPSFloat64, MAPSInt32>();
                        else if (MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterFloat32))
                            _pointcloud2_channel = new IOEltToPointCloud2<MAPSFloat64,MAPSFloat32>();
                            */

                    }
                }

                m_width = GetIntegerProperty("pointcloud2_width");
                m_height = GetIntegerProperty("pointcloud2_height");
            } // first_time

            if (m_ioeltin->VectorSize() % 3 != 0) 
			{
                Error("The input vector size is not a multiple of 3 as expected for an XYZ point cloud.");
            }
            m_ros_pointcloud2_nb_points_in = m_ioeltin->VectorSize() / 3;
            if (m_width > 0 && m_height > 0) 
			{
                if (m_ros_pointcloud2_nb_points_in != m_width * m_height) 
				{
                    MAPSStreamedString ss;
                    ss << "The input number of points (" << m_ros_pointcloud2_nb_points_in << ") does not match the dimensions of the publised PointCloud2 (" << m_width << "x" << m_height << ")";
                    Error(ss);
                }
                m_ros_pointcloud2.width = m_width;
                m_ros_pointcloud2.height = m_height;
            } 
			else 
			{
                if (m_width <= 0 && m_height <= 0) 
				{
                    m_ros_pointcloud2.width = m_ros_pointcloud2_nb_points_in;
                    m_ros_pointcloud2.height = 1;
                } else if (m_width <= 0) {
                    m_ros_pointcloud2.height = m_height;
                    m_ros_pointcloud2.width = m_ros_pointcloud2_nb_points_in/m_ros_pointcloud2.height;
                } else if (m_height <= 0) {
                    m_ros_pointcloud2.width = m_width;
                    m_ros_pointcloud2.height = m_ros_pointcloud2_nb_points_in/m_ros_pointcloud2.width;
                }
            }
            m_ros_pointcloud2.row_step = m_ros_pointcloud2.point_step * m_ros_pointcloud2.width;
            unsigned int data_size = m_ros_pointcloud2.row_step * m_ros_pointcloud2.height;
            if (m_ros_pointcloud2.data.size() != data_size) 
			{
                m_ros_pointcloud2.data.resize(m_ros_pointcloud2.row_step * m_ros_pointcloud2.height);
            }

            if (m_ros_pointcloud2_memcpy) 
			{
                if (m_ioeltin->IOEltUsedSizeInBytes(NULL) != (int)m_ros_pointcloud2.data.size()) 
				{
                    ReportError("Wrong size for received point cloud to be published to ROS. The PointCloud size should not change from one sample to another.");
                    break;
                }
                MAPS::Memcpy(&m_ros_pointcloud2.data[0],m_ioeltin->Data(),m_ros_pointcloud2.data.size());

            } 
			else 
			{
                /*
                _pointcloud2_channel->configuBuffers(&_ros_pointcloud2.data[0],m_ioeltin->Data(),_ros_pointcloud2_nb_points_in);
                CopyPointCloud2Visitor visitor;
                _pointcloud2_channel->accept(visitor);
                */
                if (m_ros_pointcloud2_output_type == 2 && MAPS::TypeFilter(m_ioeltin->Type(),MAPS::FilterInteger32) ) 
				{
                    MAPSInt32* data_in = &m_ioeltin->Integer32();
                    MAPSFloat32* data_out = (MAPSFloat32*)&m_ros_pointcloud2.data[0];
                    for (int i=m_ros_pointcloud2_nb_points_in; i>0; i--) 
					{
                        *(data_out++) = *(data_in++);
                    }
                } 
				else 
				{
                    Error("The requested data type conversion is not supported yet.");
                }
            }
            m_pub->publish(m_ros_pointcloud2);
        }
        break;
    case SENSOR_MSG_NAV_SAT_FIX:
        if (m_ioelts[0]->VectorSize() != 2) 
		{
            MAPSStreamedString ss;
            ss << "Unexpected vector size received on input navsatfix_status (" << m_ioelts[0]->VectorSize() << "). Expecting 2: fix status and service.";
            ReportError(ss);
            break;
        }
        if (m_ioelts[1]->VectorSize() != 3) 
		{
            MAPSStreamedString ss;
            ss << "Unexpected vector size received on input navsatfix_pos_lla (" << m_ioelts[1]->VectorSize() << "). Expecting 3: latitude, longitude and altitude.";
            ReportError(ss);
            break;
        }
        if (m_ioelts[2]->VectorSize() != 9) 
		{
            MAPSStreamedString ss;
            ss << "Unexpected vector size received on input navsatfix_pos_cov" << m_ioelts[2]->VectorSize() << ".) Expecting 9.";
            ReportError(ss);
            break;
        }
        if (m_ioelts[3]->VectorSize() != 1) 
		{
            MAPSStreamedString ss;
            ss << "Unexpected vector size received on input navsatfix_cov_type (" << m_ioelts[1]->VectorSize() << "). Expecting 1.";
            ReportError(ss);
            break;
        }
        {
            sensor_msgs::NavSatFix msg;
            msg.header = m_header;
            msg.status.status = m_ioelts[0]->Integer32();
            msg.status.service = m_ioelts[0]->Integer32(1);
            
            msg.latitude = m_ioelts[1]->Float64();
            msg.longitude = m_ioelts[1]->Float64(1);
            msg.altitude = m_ioelts[1]->Float64(2);
            
            for (int i=0; i<9; i++) 
			{
                msg.position_covariance[i] = m_ioelts[2]->Float64(i);
            }
            
            msg.position_covariance_type = m_ioelts[3]->Integer32();

            m_pub->publish(msg);
        }
        break;
    case SENSOR_MSG_IMU:
        if (m_ioelts[0]->VectorSize() != 4) 
		{
            MAPSStreamedString ss;
            ss << "Unexpected vector size received on input orientation_quaternion (" << m_ioelts[1]->VectorSize() << "). Expecting 4.";
            ReportError(ss);
            break;
        }
        if (m_ioelts[1]->VectorSize() != 3) 
		{
            MAPSStreamedString ss;
            ss << "Unexpected vector size received on input angular_velocities (" << m_ioelts[1]->VectorSize() << "). Expecting 3.";
            ReportError(ss);
            break;
        }
        if (m_ioelts[2]->VectorSize() != 3) 
		{
            MAPSStreamedString ss;
            ss << "Unexpected vector size received on input accelerations" << m_ioelts[2]->VectorSize() << ".) Expecting 3.";
            ReportError(ss);
            break;
        }
        {
            sensor_msgs::Imu msg;
            msg.header = m_header;
            msg.orientation.x = m_ioelts[0]->Float64();
            msg.orientation.y = m_ioelts[0]->Float64(1);
            msg.orientation.z = m_ioelts[0]->Float64(2);
            msg.orientation.w = m_ioelts[0]->Float64(3);
            msg.angular_velocity.x = m_ioelts[1]->Float64();
            msg.angular_velocity.y = m_ioelts[1]->Float64(1);
            msg.angular_velocity.z = m_ioelts[1]->Float64(2);
            msg.linear_acceleration.x = m_ioelts[2]->Float64();
            msg.linear_acceleration.y = m_ioelts[2]->Float64(1);
            msg.linear_acceleration.z = m_ioelts[2]->Float64(2);
            m_pub->publish(msg);
        }
        break;
    }
}

void MAPSros_topic_publisher::PublishGeomMsg()
{
	switch(m_message) 
	{
		case GEOM_MSG_POINT:
		{
			geometry_msgs::Point msg;
			if(m_ioeltin->VectorSize() != 3) 
			{
                ReportError("Input vector size is not as expected for a geometry_msgs::Point message. Expecting vector of 3 MAPSFloat64.");
				return;
			}
			msg.x = m_ioeltin->Float64(0);
			msg.y = m_ioeltin->Float64(1);
			msg.z = m_ioeltin->Float64(2);
			m_pub->publish(msg);
		}
		break;
		case GEOM_MSG_TWIST:
		{
			geometry_msgs::Twist msg;
			if (m_ioeltin->VectorSize() != 6) 
			{
                ReportError("Input vector size is not as expected for a geometry_msgs::Twist message. Expecting vector of 6 MAPSFloat64.");
				return;
			}
            msg.linear.x = m_ioeltin->Float64(0);
			msg.linear.y = m_ioeltin->Float64(1);
			msg.linear.z = m_ioeltin->Float64(2);
			msg.angular.x = m_ioeltin->Float64(3);
			msg.angular.y = m_ioeltin->Float64(4);
			msg.angular.z = m_ioeltin->Float64(5);
			m_pub->publish(msg);
		}
        break;
    case GEOM_MSG_TWIST_STAMPED:
        {
        geometry_msgs::TwistStamped msg;
        if (m_ioeltin->VectorSize() != 6) 
		{
            ReportError("Input vector size is not as expected for a geometry_msgs::Twist message. Expecting vector of 6 MAPSFloat64.");
            return;
        }
        msg.header = m_header;
        msg.twist.linear.x = m_ioeltin->Float64(0);
        msg.twist.linear.y = m_ioeltin->Float64(1);
        msg.twist.linear.z = m_ioeltin->Float64(2);
        msg.twist.angular.x = m_ioeltin->Float64(3);
        msg.twist.angular.y = m_ioeltin->Float64(4);
        msg.twist.angular.z = m_ioeltin->Float64(5);
        m_pub->publish(msg);
        }
        break;
	}
}

void MAPSros_topic_publisher::PublishNavMsg()
{
    switch(m_message) 
	{
        case NAV_MSG_ODOMETRY:
        {
            nav_msgs::Odometry msg;
            msg.header = m_header;
            bool error = false;
            if (m_ioelts[0]->VectorSize() != 3) 
			{
                ReportError("Input vector size on input_pose_position is not as expected. Expecting vector of 3 MAPSFloat64 (x,y,z)");
                error =true;
            }
            if(m_ioelts[1]->VectorSize() != 4) 
			{
                ReportError("Input vector size on input_pose_orientation (quaternion) is not as expected. Expecting vector of 4 MAPSFloat64 (x,y,z,w)");
                error =true;
            }
            if (m_ioelts[2]->VectorSize() != 36) 
			{
                ReportError("Input vector size on input input_pos_covariance is not as epxected. Expecting vector of 36 MAPSFloat64.");
                error =true;
            }
            if (m_ioelts[3]->VectorSize() != 6) 
			{
                ReportError("Input vector size on input input_twist is not as expected. Expecting vector of 6 MAPSFloat64 (linear x,y,z and angular x,y,z");
                error =true;
            }
            if (m_ioelts[4]->VectorSize() != 36) 
			{
                ReportError("Input vector size on input input_twist_covariance is not as expected. Expecting vector of 36 MAPSFloat64");
                error =true;
            }
            if (error) 
			{
                return;
            }
            MAPSString child_frame_id = GetStringProperty("odom_child_frame_id");
            if (child_frame_id.Len() > 0)
                msg.child_frame_id = (const char*)child_frame_id;
            msg.pose.pose.position.x = m_ioelts[0]->Float64();
            msg.pose.pose.position.y = m_ioelts[0]->Float64(1);
            msg.pose.pose.position.z = m_ioelts[0]->Float64(2);
            msg.pose.pose.orientation.x = m_ioelts[1]->Float64(0);
            msg.pose.pose.orientation.y = m_ioelts[1]->Float64(1);
            msg.pose.pose.orientation.z = m_ioelts[1]->Float64(2);
            msg.pose.pose.orientation.w = m_ioelts[1]->Float64(3);
            double* cov_out = msg.pose.covariance.data();
            double* cov_in =  &m_ioelts[2]->Float64();
            MAPS::Memcpy(cov_out,cov_in,36*sizeof(double));
            msg.twist.twist.linear.x = m_ioelts[3]->Float64(0);
            msg.twist.twist.linear.y = m_ioelts[3]->Float64(1);
            msg.twist.twist.linear.z = m_ioelts[3]->Float64(2);
            msg.twist.twist.angular.x = m_ioelts[3]->Float64(3);
            msg.twist.twist.angular.y = m_ioelts[3]->Float64(4);
            msg.twist.twist.angular.z = m_ioelts[3]->Float64(5);
            cov_out = msg.twist.covariance.data();
            cov_in = &m_ioelts[4]->Float64();
            MAPS::Memcpy(cov_out,cov_in,36*sizeof(double));
            m_pub->publish(msg);
        }
        break;
    }
}

void MAPSros_topic_publisher::PublishRmpMsg()
{
    switch(m_message)
    {
        case RMP_MSG_BOOL_STAMPED:
        {
            rmp_msgs::BoolStamped bool_stamped;
            bool_stamped.header = m_header;
            bool_stamped.data = *(m_ioeltin->Stream8());
            m_pub->publish(bool_stamped);
        }
            break;
        case RMP_MSG_AUDIO_COMMAND:
        {
            rmp_msgs::AudioCommand audio_command;
            audio_command.header = m_header;
            audio_command.command = *(m_ioeltin->Stream32());
            m_pub->publish(audio_command);
        }
            break;

    }
}


void MAPSros_topic_publisher::Death()
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

    if (m_pointcloud2_channel) 
	{
        MAPS_SAFE_DELETE(m_pointcloud2_channel);
    }
}
