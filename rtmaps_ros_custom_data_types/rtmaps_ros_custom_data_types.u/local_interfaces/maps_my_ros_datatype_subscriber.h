////////////////////////////////
// RTMaps SDK Component header
////////////////////////////////

#ifndef _Maps_my_data_types_component_H
#define _Maps_my_data_types_component_H

// Includes maps sdk library header
#include "maps.hpp"
#include "maps_ros_defines.h"
#include "maps_ros_utils.h"
#include <my_data_types/my_data_type.h>
#include <sstream>

// Declares a new MAPSComponent child class
class MAPSmy_ros_datatype_subscriber : public MAPSComponent
{
	// Use standard header definition macro
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSmy_ros_datatype_subscriber)

private :
	// Place here your specific methods and attributes
    MAPSRosUtils* _ros;
    ros::NodeHandle* _n;
    ros::Subscriber* _sub;

	bool _first_time;
	int	_message;
	int _buffsize_out;
	bool _ros_header_avail;
	bool _transfer_ros_timestamp;


    void ROSDataReceivedCallback(const my_data_types::my_data_typeConstPtr& message);

};

#endif
