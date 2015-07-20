//Copyright 2014 Giacomo Dabisias
//
//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.
//This is preliminary software and/or hardware and APIs are preliminary and subject to change.
#pragma once
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "GL/glew.h"
//#include <libfreenect2/opengl.h>
#include <signal.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/registration.h>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/rgb_packet_stream_parser.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <sys/time.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/organized_edge_detection.h>
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool shut_down = false; 
bool deb = true;



BOOST_SERIALIZATION_SPLIT_FREE(::cv::Mat)
namespace boost {
namespace serialization {
 
    /** Serialization support for cv::Mat */
    template <class Archive>
    void save(Archive & ar, const ::cv::Mat & m, const unsigned int version)
    {
		size_t elem_size = m.elemSize();
		size_t elem_type = m.type();

		ar & m.cols;
		ar & m.rows;
		ar & elem_size;
		ar & elem_type;

		const size_t data_size = m.cols * m.rows * elem_size;
		ar & boost::serialization::make_array(m.ptr(), data_size);
    }
 
    /** Serialization support for cv::Mat */
    template <class Archive>
    void load(Archive & ar, ::cv::Mat& m, const unsigned int version)
    {
		int cols, rows;
		size_t elem_size, elem_type;

		ar & cols;
		ar & rows;
		ar & elem_size;
		ar & elem_type;

		m.create(rows, cols, elem_type);

		size_t data_size = m.cols * m.rows * elem_size;
		ar & boost::serialization::make_array(m.ptr(), data_size);
    }
 
}

}
void sigintHandler(int s)
{
	shut_down = true;
}

namespace Kinect2Grabber {

template <typename PointT>
class Frame;

template <typename PointT>
class CvFrame;

template< typename PointT>
class Kinect2Grabber
{
friend class Frame<PointT>;
friend class CvFrame<PointT>;
public:

	Kinect2Grabber( ): 
					serialize_(false)
	{

		//glfwInit();
		dev_ = freenect2_.openDefaultDevice();
		
		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
		if(dev_ == 0){
			std::cout << "no device connected or failure opening the default one!" << std::endl;
			exit(1);
		}
		signal(SIGINT,sigintHandler);
		shut_down = false;
		dev_->setColorFrameListener(listener_);
		dev_->setIrAndDepthFrameListener(listener_);
		dev_->start();

		ir_camera_params_ = dev_->getIrCameraParams();
  		rgb_camera_params_ = dev_->getColorCameraParams();
		registration_ = new libfreenect2::Registration(&ir_camera_params_, &rgb_camera_params_);

		std::cout << "device initialized" << std::endl;
		std::cout << "device serial: " << dev_->getSerialNumber() << std::endl;
		std::cout << "device firmware: " << dev_->getFirmwareVersion() << std::endl;
		init();

	}


	~Kinect2Grabber(){
		this->shutDown();
		if(file_streamer_->is_open())
			file_streamer_->close();
	}

	void
	setDistance(const float dist){
		distance_ = dist;
	}

	libfreenect2::FrameMap *
	getRawFrames() {
		//using namespace std::chrono;
		//static high_resolution_clock::time_point last;

		// auto tnow = high_resolution_clock::now();
		listener_->waitForNewFrame(frames_);
		// auto tpost = high_resolution_clock::now();
		 //std::cout << "delta " << duration_cast<duration<double>>(tpost-last).count()*1000 << " " << duration_cast<duration<double>>(tpost-tnow).count()*1000 << std::endl;
		 //last = tpost;
	return &frames_;
	}

	void
	shutDown(){
		dev_->stop();
		dev_->close();
	}

	cv::Mat
	getCameraMatrixColor() const {
		return rgb_camera_params_;
	}

	cv::Mat
	getCameraMatrixDepth() const {
		return ir_camera_params_;
	}

	void
	freeFrames(){
		listener_->release(frames_);
	}

	void
	enableSerialization(){
		serialize_ = true;
	}

	void
	disableSerialization(){
		serialize_ = false;
	}


	typename pcl::PointCloud<PointT>::Ptr
	getCloud(){
		typename pcl::PointCloud<PointT>::Ptr cloud_(new pcl::PointCloud<PointT>());
		cloud_->is_dense = false;
		cloud_->points.resize(512 * 424);
		
		frames_ =  *getRawFrames();
		rgb_ = frames_[libfreenect2::Frame::Color];
		depth_ = frames_[libfreenect2::Frame::Depth];

		depthMat_ = cv::Mat(depth_->height, depth_->width, CV_32FC1, depth_->data);
	    RGBMat_ = cv::Mat(rgb_->height, rgb_->width, CV_8UC3, rgb_->data);
		createCloud(depthMat_, RGBMat_, cloud_);
		listener_->release(frames_);
		return cloud_;
	}


private:

	libfreenect2::Frame *
	getRgbFrame() {
		listener_->waitForNewFrame(frames_);
		return frames_[libfreenect2::Frame::Color];
	}

	libfreenect2::Frame *
	getIrFrame() {
		listener_->waitForNewFrame(frames_);
		return frames_[libfreenect2::Frame::Ir];
	} 

	libfreenect2::Frame *
	getDepthFrame() {
		listener_->waitForNewFrame(frames_);
		return frames_[libfreenect2::Frame::Depth];
	}

	void
	init(){
		unsigned threads = omp_get_max_threads();
		distance_ = 10000;
		depth2world_(0,0) = ir_camera_params_.fx;
		depth2world_(0,1) = 0;
		depth2world_(0,2) = ir_camera_params_.cx;
		depth2world_(0,3) = 0;
		depth2world_(1,0) = 0;
		depth2world_(1,1) = ir_camera_params_.fy;
		depth2world_(1,2) = ir_camera_params_.cy;
		depth2world_(1,3) = 0;
		depth2world_(2,0) = 0;
		depth2world_(2,1) = 0;
		depth2world_(2,2) = 1;
		depth2world_(2,3) = 0;
		depth2world_(3,0) = 0;
		depth2world_(3,1) = 0;
		depth2world_(3,2) = 0;
		depth2world_(3,3) = 1;
		depth2world_ = depth2world_.inverse();
	}

	void 
	createCloud(const cv::Mat & depth, const cv::Mat & color, typename pcl::PointCloud<PointT>::Ptr & cloud) 
	{
		
		std::cout << "called" << std::endl;
		for (int y = 0; y < 424; y++){
	        for (int x = 0; x < 512; x++) {
	        	PointT *itP = &cloud->points[y * 512 + x];
	            float cx;
	            float cy;
	            float z = depth.at<float>(y, x);
	            float depth = z / 1000.0f;
	            
	            if (z == 0 )
	            {
	                itP->x = itP->y = itP->z = std::numeric_limits<float>::quiet_NaN();
					itP->rgba = 0;
					continue;
	            }
	            
	            registration_->apply(x, y, z, cx, cy);

	            if (cx > 0 && cy > 0 && cx < 1920 && cy < 1080){

	            	Eigen::Vector4d psd(x, y, 1.0, 1.0 / depth);
					Eigen::Vector4d psddiv = psd * depth;
					Eigen::Vector4d pworld = depth2world_ * psddiv;

	            	itP->z = depth;
					itP->x = pworld.x();
					itP->y = pworld.y();

	                const cv::Vec3b tmp = color.at<cv::Vec3b>(cy, cx);
					itP->b = tmp.val[0];
					itP->g = tmp.val[1];
					itP->r = tmp.val[2];
				}
	        }
        }
	}



	void
	serializeFrames(const cv::Mat & depth, const cv::Mat & color)
	{	
		struct timeval tv;
    	gettimeofday(&tv,NULL);
    	uint64_t now = tv.tv_sec*(uint64_t)1000000 + tv.tv_usec;
		if(file_streamer_ == 0){
			file_streamer_ = new std::ofstream();
			file_streamer_->open ("stream", std::ios::binary);
			oa_ = new boost::archive::binary_oarchive(*file_streamer_);
		}

		(*oa_) << now << depth << color;
	}




	libfreenect2::Freenect2 freenect2_;
	libfreenect2::Registration * registration_ = 0;
	libfreenect2::Freenect2Device * dev_ = 0;
	libfreenect2::SyncMultiFrameListener * listener_ = 0;
	libfreenect2::Frame * depth_ = 0;
	libfreenect2::Frame *rgb_ = 0;
	libfreenect2::FrameMap frames_;
	cv::Mat depthMat_, RGBMat_;
  	bool serialize_;
  	float distance_;
  	std::ofstream * file_streamer_ = 0;
  	boost::archive::binary_oarchive * oa_ = 0;
  	Eigen::Matrix4d depth2world_ ;
  	libfreenect2::Freenect2Device::IrCameraParams ir_camera_params_;
    libfreenect2::Freenect2Device::ColorCameraParams rgb_camera_params_;
  	

};

template <typename PointT>
class Frame{
public:

	libfreenect2::Frame * data_;

	Frame(Kinect2Grabber<PointT> & k): grabber_(k){
		data_ = grabber_.getRgbFrame();
	}

	~Frame(){
		grabber_.freeFrames();
	}

private:
	Kinect2Grabber<PointT> & grabber_; 
};

template <typename PointT>
class CvFrame{
public:

	cv::Mat data_;

	CvFrame(Kinect2Grabber<PointT> & k): grabber_(k){
		freenect_data_ = grabber_.getRgbFrame();
		data_ = cv::Mat(freenect_data_->height, freenect_data_->width, CV_8UC3, freenect_data_->data);
	}

	~CvFrame(){
		grabber_.freeFrames();
	}

private:
	Kinect2Grabber<PointT> & grabber_; 
	libfreenect2::Frame * freenect_data_;
};


}

