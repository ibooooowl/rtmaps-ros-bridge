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

#ifndef __maps_ros_defines_h__
#define __maps_ros_defines_h__

#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int64MultiArray.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "can_msgs/Frame.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Joy.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "rmp_msgs/AudioCommand.h"
#include "rmp_msgs/BoolStamped.h"
#include "rmp_msgs/Battery.h"
#include "rmp_msgs/FaultStatus.h"
#include "rmp_msgs/MotorStatus.h"

static const char* s_topic_types[] = {
		"std_msgs",
		"sensor_msgs",
		"geometry_msgs",
		"nav_msgs",
		"stereo_msgs",
		"trajectory_msgs",
		"visualization_msgs",
		"actionlib_msgs",
        "diagnostic_msgs",
        "can_msgs",
        "rmp_msgs"
};

#define TOPIC_TYPE_STD 			0
#define TOPIC_TYPE_SENSOR 		1
#define TOPIC_TYPE_GEOM 		2
#define TOPIC_TYPE_NAV			3
#define TOPIC_TYPE_STEREO		4
#define TOPIC_TYPE_TRAJ			5
#define TOPIC_TYPE_VISU			6
#define TOPIC_TYPE_ACTION		7
#define TOPIC_TYPE_DIAG			8
#define TOPIC_TYPE_CAN          9
#define TOPIC_TYPE_RMP          10

static const char* s_std_msgs[] = {
		"Int 32",
		"Int 32 array",
		"Int 64",
		"Int 64 array",
		"Float 32",
		"Float 32 array",
		"Float 64",
		"Float 64 array",
		"Text"
};

#define STD_MSG_INT32 			0
#define STD_MSG_INT32_ARRAY		1
#define STD_MSG_INT64			2
#define STD_MSG_INT64_ARRAY		3
#define STD_MSG_FLOAT32			4
#define STD_MSG_FLOAT32_ARRAY	5
#define STD_MSG_FLOAT64			6
#define STD_MSG_FLOAT64_ARRAY	7
#define STD_MSG_TEXT			8


static const char* s_sensor_msgs[] = {
//		"CameraInfo",
//		"ChannelFloat32",
		"CompressedImage",
		"Image",
		"Imu",
		"JointState",
		"Joy",
		"JoyFeedback",
		"JoyFeedbackArray",
		"LaserScan",
		"NavSatFix",
		"NavSatStatus",
		"PointCloud",
		"PointCloud2",
		"PointField",
		"Range",
		"RegionOfInterest"
};

//#define SENSOR_MSG_CAMERA_INFO				0
//#define SENSOR_MSG_CHANNEL_FLOAT32			1
#define SENSOR_MSG_COMPRESSED_IMAGE			0
#define SENSOR_MSG_IMAGE					1
#define SENSOR_MSG_IMU						2
#define SENSOR_MSG_JOINT_STATE				3
#define SENSOR_MSG_JOY						4
#define SENSOR_MSG_JOY_FEEDBACK				5
#define SENSOR_MSG_JOY_FEEDBACK_ARRAY		6
#define SENSOR_MSG_LASER_SCAN				7
#define SENSOR_MSG_NAV_SAT_FIX				8
#define SENSOR_MSG_NAV_SAT_STATUS			9
#define SENSOR_MSG_POINT_CLOUD				10
#define SENSOR_MSG_POINT_CLOUD2				11
#define SENSOR_MSG_POINT_FIELD				12
#define SENSOR_MSG_RANGE					13
//#define SENSOR_MSG_ROI						14

static const char* s_geometry_msgs[] = {
		"Point",
		"Point32",
		"PointStamped",
		"Polygon",
		"PolygonStamped",
		"Pose",
		"Pose2D",
		"PoseArray",
		"PoseStamped",
		"PoseWithCovariance",
		"Quaternion",
		"QuaternionStamped",
		"Transform",
		"TransformStamped",
		"Twist",
		"TwistStamped",
		"TwistWithCovariance",
		"TwistWidthCovarianceStamped",
		"Vector3",
		"Vector3Stamped",
		"Wrench",
		"WrenchStamped"
};

#define GEOM_MSG_POINT					0
#define GEOM_MSG_POINT32				1
#define GEOM_MSG_POINT_STAMPED			2
#define GEOM_MSG_POLYGON				3
#define GEOM_MSG_POLYGON_STAMPED		4
#define GEOM_MSG_POSE					5
#define GEOM_MSG_POSE_2D				6
#define GEOM_MSG_POSE_ARRAY				7
#define GEOM_MSG_POSE_STAMPED			8
#define GEOM_MSG_POSE_WITH_COV			9
#define GEOM_MSG_QUATERNION				10
#define GEOM_MSG_QUATERNION_STAMPED		11
#define GEOM_MSG_TRANSFORM				12
#define GEOM_MSG_TRANSFORM_STAMPED		13
#define GEOM_MSG_TWIST					14
#define GEOM_MSG_TWIST_STAMPED			15
#define GEOM_MSG_TWIST_WITH_COV			16
#define GEOM_MSG_TWIST_WITH_COV_STAMPED	17
#define GEOM_MSG_VECTOR3				18
#define GEOM_MSG_VECTOR3_STAMPED		19
#define GEOM_MSG_WRENCH					20
#define GEOM_MSG_WRENCH_STAMPED			21

static const char* s_nav_msgs[] = {
        "GridCells",
        "MapMetaData",
        "OccupancyGrid",
        "Odometry",
        "Path"
};

#define NAV_MSG_GRIDCELLS               0
#define NAV_MSG_MAP_METADATA            1
#define NAV_MSG_OCCUPANCY_GRID          2
#define NAV_MSG_ODOMETRY                3
#define NAV_MSG_PATH                    4


static const char* s_visu_msgs[] = {
        "Marker",
        "MarkerArray"
};

#define VISU_MSG_MARKER        0
#define VISU_MSG_MARKER_ARRAY  1

static const char* s_rmp_msgs[] = {
        "MotorStatus",
        "Battery",
        "FaultStatus",
        "Deadman",
        "AudioCommand"
};
#define RMP_MSG_MOTOR_STATUS        0
#define RMP_MSG_BATTERY             1
#define RMP_MSG_FAULT_STATUS        2
#define RMP_MSG_BOOL_STAMPED        3
#define RMP_MSG_AUDIO_COMMAND       4


#define MAX_ARRAY_LAYOUT_LABEL_SIZE	256
#define MAX_ARRAY_LAYOUT_DIMENSIONS	3

#define DATA_TYPE_TEXT 			        0
#define DATA_TYPE_IMG			        1
#define DATA_TYPE_INT32			        2
#define DATA_TYPE_INT32_ARRAY	        3
#define DATA_TYPE_INT64			        4
#define DATA_TYPE_INT64_ARRAY	        5
#define DATA_TYPE_FLOAT32		        6
#define DATA_TYPE_FLOAT32_ARRAY	        7
#define DATA_TYPE_FLOAT64		        8
#define DATA_TYPE_FLOAT64_ARRAY	        9
#define DATA_TYPE_LASER_SCAN	        10
#define DATA_TYPE_TWIST			        11
#define DATA_TYPE_UINT32			    12

typedef struct ROSArrayLayoutDim
{
	char label[MAX_ARRAY_LAYOUT_LABEL_SIZE];
	int	 size;
	int  stride;
} ROSArrayLayoutDim;

typedef struct ROSArrayLayout
{
	int data_offset;
	int nb_dims;
	ROSArrayLayoutDim dim[MAX_ARRAY_LAYOUT_DIMENSIONS];
}ROSArrayLayout;

static const char* s_can_msgs[] = {
        "Frame"
};

#define CAN_MSG_FRAME 0

const MAPSTypeFilterBase MAPSFilterROSArrayLayout=MAPSFilterUserStructure(ROSArrayLayout);


#endif //__maps_ros_defines_h__
