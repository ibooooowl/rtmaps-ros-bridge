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

#ifndef _Maps_ros_topic_publisher_H
#define _Maps_ros_topic_publisher_H

// Includes maps sdk library header
#include "maps.hpp"
#include "maps_ros_utils.h"
#include "maps_ros_defines.h"
#include <cstdio>
#include <cstring>
#include <vector>
class AbstractIOEltToPointCloud2;
template <typename OUT_TYPE, typename IN_TYPE> class IOEltToPointCloud2;

class AbstractPointCloud2Visitor
{
public:
    virtual ~AbstractPointCloud2Visitor() {}

    virtual void visit(AbstractIOEltToPointCloud2&) {}
    virtual void visit(IOEltToPointCloud2<MAPSInt32,MAPSFloat32>&);
    virtual void visit(IOEltToPointCloud2<MAPSInt32,MAPSFloat64>&);
    virtual void visit(IOEltToPointCloud2<MAPSFloat32,MAPSInt32>&);
    virtual void visit(IOEltToPointCloud2<MAPSFloat32,MAPSFloat64>&);
    virtual void visit(IOEltToPointCloud2<MAPSFloat64,MAPSInt32>&);
    virtual void visit(IOEltToPointCloud2<MAPSFloat64,MAPSFloat32>&);
};

class CopyPointCloud2Visitor : public AbstractPointCloud2Visitor
{
public:
    template <typename OUT_TYPE, typename IN_TYPE>
    void CopyToPointCloud2(IOEltToPointCloud2<OUT_TYPE, IN_TYPE>& chan);

    void visit(IOEltToPointCloud2<MAPSInt32,MAPSFloat32>& chan) {CopyToPointCloud2 (chan);}
    void visit(IOEltToPointCloud2<MAPSInt32,MAPSFloat64>& chan) {CopyToPointCloud2 (chan);}
    void visit(IOEltToPointCloud2<MAPSFloat32,MAPSInt32>& chan) {CopyToPointCloud2 (chan);}
    void visit(IOEltToPointCloud2<MAPSFloat32,MAPSFloat64>& chan) {CopyToPointCloud2 (chan);}
    void visit(IOEltToPointCloud2<MAPSFloat64,MAPSInt32>& chan) {CopyToPointCloud2 (chan);}
    void visit(IOEltToPointCloud2<MAPSFloat64,MAPSFloat32>& chan) {CopyToPointCloud2 (chan);}

};

template <typename OUT_TYPE, typename IN_TYPE>
void CopyPointCloud2Visitor::CopyToPointCloud2(IOEltToPointCloud2<OUT_TYPE, IN_TYPE>& chan)
{
    for (int i=chan.m_count; i>0; i--)
    {
        *(chan.m_out_ptr++) = *(chan.m_in_ptr++);
    }
}

class AbstractIOEltToPointCloud2
{
public:
    virtual ~AbstractIOEltToPointCloud2() {}

    virtual void accept (AbstractPointCloud2Visitor&);
    virtual void configuBuffers(void* out_ptr, void* in_ptr, int count);
};

template <typename OUT_TYPE, typename IN_TYPE>
class IOEltToPointCloud2 : public AbstractIOEltToPointCloud2
{
    typedef OUT_TYPE out_value_type;
    typedef IN_TYPE in_value_type;
public:
    IOEltToPointCloud2() {}
    out_value_type*     m_out_ptr;
    in_value_type*      m_in_ptr;
    int                 m_count;

    virtual void configuBuffers(void* out_ptr, void* in_ptr, int count) {m_out_ptr =  static_cast<out_value_type*>(out_ptr); m_in_ptr = static_cast<in_value_type*>(in_ptr);m_count = count;}
    virtual void accept(AbstractPointCloud2Visitor& visitor) {visitor.visit((*this));}
};



// Declares a new MAPSComponent child class
class MAPSros_topic_publisher : public MAPSComponent
{
	// Use standard header definition macro
	MAPS_COMPONENT_HEADER_CODE_WITHOUT_CONSTRUCTOR(MAPSros_topic_publisher)
	MAPSros_topic_publisher(const char* name, MAPSComponentDefinition& cd);
	void 			Dynamic();

	int CreateIOsForStdTopics(bool* output_header);
	int CreateIOsForSensorTopics(bool* output_header);
	int CreateIOsForGeomTopics(bool* output_header);
    int CreateIOsForNavTopics(bool* output_header);
    int CreateIOsForRmpTopics(bool* output_header);
private :
	// Place here your specific methods and attributes
 	MAPSROSBridgeCoreFunctionInterface* m_ros_bridge_cf;
	ros::NodeHandle* 	m_n;
	ros::Publisher* 	m_pub;

	int 				m_topic_type;
	int					m_message;

	int 				m_nb_inputs;
	MAPSIOElt*			m_ioeltin;
    MAPSInput*			m_inputs[5];
    MAPSIOElt*			m_ioelts[5];

	bool				m_output_header;
	bool				m_publish_rtmaps_timestamp;
	std_msgs::Header 	m_header; //!< ROS header

	void PublishStdMsg();
	void PublishSensorMsg();
	void PublishGeomMsg();
    void PublishNavMsg();
    void PublishRmpMsg();

	sensor_msgs::CompressedImage    m_ros_comp_img;
	sensor_msgs::Image 		        m_ros_img;
	sensor_msgs::LaserScan 	        m_ros_laser_scan;
    AbstractIOEltToPointCloud2*     m_pointcloud2_channel;
    bool                            m_ros_pointcloud2_memcpy;
    int                             m_ros_pointcloud2_output_type;
    int                             m_ros_pointcloud2_nb_points_in;
    int                             m_width;
    int                             m_height;
    sensor_msgs::PointCloud2        m_ros_pointcloud2;

	bool    m_first_time;
	int     m_count;
	bool    m_laser_supports_intens;
};



#endif
