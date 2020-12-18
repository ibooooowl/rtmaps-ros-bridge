////////////////////////////////
// RTMaps SDK Component header
////////////////////////////////

#ifndef _Maps_ros_topic_publisher_H
#define _Maps_ros_topic_publisher_H

// Includes maps sdk library header
#include "maps.hpp"
#include "maps_ros_defines.h"
#include "maps_ros_utils.h"
#include <my_data_types/my_data_type.h>

// Declares a new MAPSComponent child class
class MAPSmy_ros_datatype_publisher : public MAPSComponent
{
	// Use standard header definition macro
    MAPS_COMPONENT_STANDARD_HEADER_CODE(MAPSmy_ros_datatype_publisher)
private :
	// Place here your specific methods and attributes
    MAPSRosUtils*       _ros;
	ros::NodeHandle* 	_n;
	ros::Publisher* 	_pub;


    MAPSIOElt*          _ioeltin;

    bool                _first_time;
    int                 _count;
	bool				_publish_rtmaps_timestamp;
	std_msgs::Header 	_header; //!< ROS header
    my_data_types::my_data_type _my_data_type;

    void PublishMyMsg();

};

#endif
