////////////////////////////////
// RTMaps SDK Component header
////////////////////////////////

#ifndef _Maps_ros_topic_publisher_H
#define _Maps_ros_topic_publisher_H

// Includes maps sdk library header
#include "maps.hpp"
#include "maps_ros_defines.h"
#include "maps_ros_utils.h"

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
private :
	// Place here your specific methods and attributes
    MAPSRosUtils*       _ros;
	ros::NodeHandle* 	_n;
	ros::Publisher* 	_pub;

	int 				_topic_type;
	int					_message;

	int 				_nb_inputs;
	MAPSIOElt*			_ioeltin;
    MAPSInput*			_inputs[5];
    MAPSIOElt*			_ioelts[5];

	bool				_output_header;
	bool				_publish_rtmaps_timestamp;
	std_msgs::Header 	_header; //!< ROS header

	void PublishStdMsg();
	void PublishSensorMsg();
	void PublishGeomMsg();
    void PublishNavMsg();

	sensor_msgs::CompressedImage _ros_comp_img;
	sensor_msgs::Image 		_ros_img;
	sensor_msgs::LaserScan 	_ros_laser_scan;
    AbstractIOEltToPointCloud2*         _pointcloud2_channel;
    bool                    _ros_pointcloud2_memcpy;
    int                     _ros_pointcloud2_output_type;
    int                     _ros_pointcloud2_nb_points_in;
    int                     _width;
    int                     _height;
    sensor_msgs::PointCloud2 _ros_pointcloud2;

	bool _first_time;
	int _count;
	bool _laser_supports_intens;
};

#endif
