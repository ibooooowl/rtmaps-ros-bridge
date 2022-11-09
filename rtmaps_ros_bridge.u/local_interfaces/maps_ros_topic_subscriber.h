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

#ifndef _Maps_ros_topic_subscriber_H
#define _Maps_ros_topic_subscriber_H

// Includes maps sdk library header
#include "maps.hpp"
#include "maps_types.h"
#include "maps_ros_defines.h"
#include "maps_ros_utils.h"
#include <ros/callback_queue.h>

#include <sstream>
#include <map>

#define MARKER_OUTPUT_COUNT                 12

#define MARKER_OUTPUT_NB_ARROW              0
#define MARKER_OUTPUT_NB_CUBE               1
#define MARKER_OUTPUT_NB_SPHERE             2
#define MARKER_OUTPUT_NB_CYLINDER           3
#define MARKER_OUTPUT_NB_LINE_STRIP         4
#define MARKER_OUTPUT_NB_LINE_LIST          5
#define MARKER_OUTPUT_NB_CUBE_LIST          6
#define MARKER_OUTPUT_NB_SPHERE_LIST        7
#define MARKER_OUTPUT_NB_POINTS             8
#define MARKER_OUTPUT_NB_TEXT_VIEW_FACING   9
#define MARKER_OUTPUT_NB_MESH_RESOURCE      10
#define MARKER_OUTPUT_NB_TRIANGLE_LIST      11


#define MARKER_OUTPUT_SIZE_MAX_ARROW         512
#define MARKER_OUTPUT_SIZE_MAX_CUBE          512
#define MARKER_OUTPUT_SIZE_MAX_SPHERE        512
#define MARKER_OUTPUT_SIZE_MAX_CYLINDER      512
#define MARKER_OUTPUT_SIZE_MAX_LINE_STRIP    1024*3
#define MARKER_OUTPUT_SIZE_MAX_LINE_LIST     1024*3
#define MARKER_OUTPUT_SIZE_MAX_POINTS        1024
#define MARKER_OUTPUT_SIZE_MAX_TRIANGLE_LIST 512


// Accumulate markers by namespace and id
class MarkersCache 
{
public:
    MarkersCache() { }
    typedef visualization_msgs::Marker::ConstPtr MarkerConstPtr;

    int apply_action(const MarkerConstPtr marker) 
    {
        int concerned_output = GetMarkerOutputByType(marker);
        switch (marker->action) 
        {
            case visualization_msgs::Marker::DELETEALL:
                for (auto& c : m_markers_cache)
                    c.clear();
                break;
            case visualization_msgs::Marker::DELETE:
                remove(concerned_output, marker);
                break;
            case visualization_msgs::Marker::ADD: {
                add(concerned_output, marker);
                break;
            }
            default:
                break;
        }
        return concerned_output;
    }
    void add(int concerned_output, const MarkerConstPtr& m) 
    {
        auto& map = m_markers_cache[concerned_output];
        map[get_key(m)] = boost::make_shared< visualization_msgs::Marker >(*m);
    }
    void remove(int concerned_output, const MarkerConstPtr& m) 
    {
        auto& map = m_markers_cache[concerned_output];
        map.erase(get_key(m));
    }
    std::string get_key(const MarkerConstPtr& m) 
    {
        std::string ret = m->ns + "##" + std::to_string(m->id);
        return ret;
    }
    std::vector< MarkerConstPtr > get_markers_by_output(int concerned_output) 
    {
        std::vector< MarkerConstPtr > ret;
        auto& map = m_markers_cache[concerned_output];
        for (auto& e : map) 
        {
            ret.push_back(e.second);
        }
        return ret;
    }
    void clear() 
    {
        for (auto& e : m_markers_cache)
            e.clear();
    }
    int GetMarkerOutputByType(const visualization_msgs::Marker::ConstPtr& marker) 
    {
        // get concerned output
        int concerned_output = 0;
        switch (marker->type) 
        {
            case visualization_msgs::Marker::ARROW :
                concerned_output = MARKER_OUTPUT_NB_ARROW;
                break;
            case visualization_msgs::Marker::CUBE :
                concerned_output = MARKER_OUTPUT_NB_CUBE;
                break;
            case visualization_msgs::Marker::SPHERE :
                concerned_output = MARKER_OUTPUT_NB_SPHERE;
                break;
            case visualization_msgs::Marker::CYLINDER :
                concerned_output = MARKER_OUTPUT_NB_CYLINDER;
                break;
            case visualization_msgs::Marker::POINTS :
                concerned_output = MARKER_OUTPUT_NB_POINTS;
                break;
            case visualization_msgs::Marker::LINE_STRIP :
                concerned_output = MARKER_OUTPUT_NB_LINE_STRIP;
                break;
            case visualization_msgs::Marker::LINE_LIST :
                concerned_output = MARKER_OUTPUT_NB_LINE_LIST;
                break;
            case visualization_msgs::Marker::SPHERE_LIST :
                concerned_output = MARKER_OUTPUT_NB_SPHERE_LIST;
                break;
            case visualization_msgs::Marker::TEXT_VIEW_FACING :
                concerned_output = MARKER_OUTPUT_NB_TEXT_VIEW_FACING;
                break;
            case visualization_msgs::Marker::MESH_RESOURCE :
                concerned_output = MARKER_OUTPUT_NB_MESH_RESOURCE;
                break;
            case visualization_msgs::Marker::TRIANGLE_LIST :
                concerned_output = MARKER_OUTPUT_NB_TRIANGLE_LIST;
                break;
            default:
                break;
        }
        return concerned_output;
    }

private:
    std::map< std::string, visualization_msgs::Marker::ConstPtr > m_markers_cache[MARKER_OUTPUT_COUNT];
};

// Declares a new MAPSComponent child class
class MAPSros_topic_subscriber : public MAPSComponent
{
	// Use standard header definition macro
	MAPS_COMPONENT_HEADER_CODE_WITHOUT_CONSTRUCTOR(MAPSros_topic_subscriber)
	MAPSros_topic_subscriber(const char* name, MAPSComponentDefinition& cd);
	void Dynamic();

	void CreateIOsForStdTopics(bool* ros_header_avail);
	void CreateIOsForSensorTopics(bool* ros_header_avail);
	void CreateIOsForGeomTopics(bool* ros_header_avail);
    void CreateIOsForNavTopics(bool* ros_header_avail);
    void CreateIOsForVisuTopics(bool* ros_header_avail);
    void CreateIOsForCANTopics(bool* ros_header_avail);
    void AllocateOutputsForVisuTopics();

	// Place here your specific methods and attributes
	ros::NodeHandle* m_n;
	ros::Subscriber* m_sub;
    //ros::CallbackQueue _cb_queue;

	bool m_first_time;
	int m_topic_type;
	int	m_message;
	int m_buffsize_out;
	bool m_ros_header_avail;
	bool m_transfer_ros_timestamp;

	int m_nb_laser_scan_points;
	int m_nb_laser_intens_data;
	bool m_discard_out_of_range;

	int m_nb_points;
	int m_nb_channels;
	int m_nb_joy_axes;
	int m_nb_joy_buttons;	

	int m_nb_fields;
    int m_point_step;

    // Markers Cache: array of cached markers per output
    MarkersCache m_markers_cache;

	void ROSStringReceivedCallback(const std_msgs::StringConstPtr& message);
	void ROSInt32ReceivedCallback(const std_msgs::Int32ConstPtr& msg);
	void ROSInt32ArrayReceivedCallback(const std_msgs::Int32MultiArrayConstPtr& msg);
	void ROSInt64ReceivedCallback(const std_msgs::Int64ConstPtr& msg);
	void ROSInt64ArrayReceivedCallback(const std_msgs::Int64MultiArrayConstPtr& msg);
	void ROSFloat32ReceivedCallback(const std_msgs::Float32ConstPtr& msg);
	void ROSFloat32ArrayReceivedCallback(const std_msgs::Float32MultiArrayConstPtr& msg);
	void ROSFloat64ReceivedCallback(const std_msgs::Float64ConstPtr& msg);
	void ROSFloat64ArrayReceivedCallback(const std_msgs::Float64MultiArrayConstPtr& msg);

	void ROSImageReceivedCallback(const sensor_msgs::Image::ConstPtr& ros_image);
	void ROSCompressedImageReceivedCallback(const sensor_msgs::CompressedImage::ConstPtr& ros_image);
	void ROSLaserScanReceivedCallback(const sensor_msgs::LaserScan::ConstPtr& ros_laser_scan);
	void ROSPointCloudReceivedCallback(const sensor_msgs::PointCloud::ConstPtr& ros_point_cloud);
	void ROSPointCloud2ReceivedCallback(const sensor_msgs::PointCloud2::ConstPtr& ros_point_cloud2);
	void ROSJoyReceivedCallback(const sensor_msgs::Joy::ConstPtr& ros_joy);
	void ROSImuReceivedCallback(const sensor_msgs::Imu::ConstPtr& ros_imu);
	void ROSRangeReceivedCallback(const sensor_msgs::Range::ConstPtr& ros_range);
    void ROSNavSatFixReceivedCallback(const sensor_msgs::NavSatFix::ConstPtr& ros_navsatfix);

	void ROSPointReceivedCallback(const geometry_msgs::Point::ConstPtr& point);
	void ROSPoseReceivedCallback(const geometry_msgs::Pose::ConstPtr& pose);
	void ROSPoseStampedReceivedCallback(const geometry_msgs::PoseStamped::ConstPtr& posestamped);
	void ROSTwistReceivedCallback(const geometry_msgs::Twist::ConstPtr& twist);
    void ROSTwistStampedReceivedCallback(const geometry_msgs::TwistStamped::ConstPtr& twist);

    void ROSOdometryReceivedCallback(const nav_msgs::Odometry::ConstPtr& odometry);
    void ROSVisuMarkerReceivedCallback(const visualization_msgs::Marker::ConstPtr& marker);
    void ROSVisuMarkerArrayReceivedCallback(const visualization_msgs::MarkerArray::ConstPtr& markers);

    void ROSCANFrameReceivedCallback(const can_msgs::Frame::ConstPtr& frame);

	void OutputArrayLayout(const std_msgs::MultiArrayLayout* ros_layout, MAPSTimestamp t);
    void SetComponentInError();
    void OutputMarkerOnlyFill(MAPSRealObject& obj, const visualization_msgs::Marker::ConstPtr& marker, int id = 0);
    int GetMarkerOutputByType(const visualization_msgs::Marker::ConstPtr& marker);
    int UpdateMarkerCache(const visualization_msgs::Marker::ConstPtr& marker);
    void MarkersWriteByType(std::vector< visualization_msgs::Marker::ConstPtr >& markers, int32_t marker_type, int concerned_output, MAPSTimestamp marker_ts);
};
#endif
