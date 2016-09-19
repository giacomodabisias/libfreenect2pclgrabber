/*
Copyright 2015, Giacomo Dabisias"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

@Author 
Giacomo Dabisias, PhD Student
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>
#ifdef WITH_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif
#include <opencv2/opencv.hpp>
#include <signal.h>
#include <cstdlib>
#include <string>
#include <iostream>
#include <chrono>
#include <Eigen/Core>
#ifdef WITH_SERIALIZATION
#include "serialization.h"
#endif
#ifdef WITH_ROS
#include "ros_impl.h"
#endif

bool stop = false;

enum Processor{
	CPU, OPENCL, OPENGL, CUDA
};

void sigint_handler(int s)
{
	stop = true;
}

class K2G {

public:

	K2G(Processor p = CPU, bool mirror = false, std::string serial = std::string()): mirror_(mirror), listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth), 
	                                       undistorted_(512, 424, 4), registered_(512, 424, 4), big_mat_(1920, 1082, 4), qnan_(std::numeric_limits<float>::quiet_NaN()){

		signal(SIGINT,sigint_handler);

		if(freenect2_.enumerateDevices() == 0)
		{
			std::cout << "no kinect2 connected!" << std::endl;
			exit(-1);
		}

		switch (p)
		{
			case CPU:
				std::cout << "creating Cpu processor" << std::endl;
				if (serial.empty())
					dev_ = freenect2_.openDefaultDevice (new libfreenect2::CpuPacketPipeline ());
				else
					dev_ = freenect2_.openDevice (serial, new libfreenect2::CpuPacketPipeline ());
				std::cout << "created" << std::endl;
				break;
#ifdef WITH_OPENCL
			case OPENCL:
				std::cout << "creating OpenCL processor" << std::endl;
				if(serial.empty())
					dev_ = freenect2_.openDefaultDevice(new libfreenect2::OpenCLPacketPipeline());
				else
					dev_ = freenect2_.openDevice(serial, new libfreenect2::OpenCLPacketPipeline());
				break;
#endif
			case OPENGL:
				std::cout << "creating OpenGL processor" << std::endl;
				if (serial.empty())
					dev_ = freenect2_.openDefaultDevice (new libfreenect2::OpenGLPacketPipeline ());
				else
					dev_ = freenect2_.openDevice (serial, new libfreenect2::OpenGLPacketPipeline ());
				break;
#ifdef WITH_CUDA
			case CUDA:
				std::cout << "creating Cuda processor" << std::endl;
				if(serial.empty())
					dev_ = freenect2_.openDefaultDevice(new libfreenect2::CudaPacketPipeline());
				else
					dev_ = freenect2_.openDevice(serial, new libfreenect2::CudaPacketPipeline());
				break;
#endif
			default:
				std::cout << "creating Cpu processor" << std::endl;
				if (serial_.empty())
					dev_ = freenect2_.openDefaultDevice (new libfreenect2::CpuPacketPipeline ());
				else
					dev_ = freenect2_.openDevice (serial, new libfreenect2::CpuPacketPipeline ());
				break;
		}

		if(!serial.empty())
			serial_ = serial;
		else
			serial_ = freenect2_.getDefaultDeviceSerialNumber();
		
		dev_->setColorFrameListener(&listener_);
		dev_->setIrAndDepthFrameListener(&listener_);
		dev_->start();

		logger_ = libfreenect2::getGlobalLogger();

		registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

		prepareMake3D(dev_->getIrCameraParams());
#ifdef WITH_SERIALIZATION
		serialize_ = false;
		file_streamer_ = NULL;
		oa_ = NULL;
#endif
 	}

	libfreenect2::Freenect2Device::IrCameraParams getIrParameters(){
		libfreenect2::Freenect2Device::IrCameraParams ir = dev_->getIrCameraParams();
		return ir;
	}

	libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters(){
		libfreenect2::Freenect2Device::ColorCameraParams rgb = dev_->getColorCameraParams();
		return rgb;
	}

	void disableLog() {
		logger_ = libfreenect2::getGlobalLogger();
		libfreenect2::setGlobalLogger(nullptr);
	}

	void enableLog() {
		libfreenect2::setGlobalLogger(logger_);
	}

	void printParameters(){
		libfreenect2::Freenect2Device::ColorCameraParams cp = getRgbParameters();
		std::cout << "rgb fx=" << cp.fx << ",fy=" << cp.fy <<
			",cx=" << cp.cx << ",cy=" << cp.cy << std::endl;
		libfreenect2::Freenect2Device::IrCameraParams ip = getIrParameters();
		std::cout << "ir fx=" << ip.fx << ",fy=" << ip.fy <<
			",cx=" << ip.cx << ",cy=" << ip.cy <<
			",k1=" << ip.k1 << ",k2=" << ip.k2 << ",k3=" << ip.k3 <<
			",p1=" << ip.p1 << ",p2=" << ip.p2 << std::endl;
	}

	void storeParameters(){
		libfreenect2::Freenect2Device::ColorCameraParams cp = getRgbParameters();
		libfreenect2::Freenect2Device::IrCameraParams ip = getIrParameters();

		cv::Mat rgb = (cv::Mat_<float>(3,3) << cp.fx, 0, cp.cx, 0, cp.fy, cp.cy, 0, 0, 1);
		cv::Mat depth = (cv::Mat_<float>(3,3) << ip.fx, 0, ip.cx, 0, ip.fy, ip.cy, 0, 0, 1);
		cv::Mat depth_dist = (cv::Mat_<float>(1,5) << ip.k1, ip.k2, ip.p1, ip.p2, ip.k3);
		std::cout << "storing " << serial_ << std::endl;
		cv::FileStorage fs("calib_" + serial_ + ".yml", cv::FileStorage::WRITE);
	   
	    fs << "CcameraMatrix" << rgb;
	    fs << "DcameraMatrix" << depth << "distCoeffs" << depth_dist;

	    fs.release();
		
	}

#ifdef WITH_PCL
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){
		const short w = undistorted_.width;
		const short h = undistorted_.height;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));
        
		return updateCloud(cloud);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(const libfreenect2::Frame * rgb, const libfreenect2::Frame * depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
		const short w = undistorted_.width;
		const short h = undistorted_.height;
		if(cloud->size() != w * h)  
			cloud->resize(w * h);      
		return updateCloud(rgb, depth, cloud);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
		
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

		registration_->apply(rgb, depth, &undistorted_, &registered_, true, &big_mat_, map_);
		const std::size_t w = undistorted_.width;
		const std::size_t h = undistorted_.height;

        cv::Mat tmp_itD0(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
        cv::Mat tmp_itRGB0(registered_.height, registered_.width, CV_8UC4, registered_.data);
        
        if (mirror_ == true){

            cv::flip(tmp_itD0,tmp_itD0,1);
            cv::flip(tmp_itRGB0,tmp_itRGB0,1);

        }

        const float * itD0 = (float *) tmp_itD0.ptr();
        const char * itRGB0 = (char *) tmp_itRGB0.ptr();
        
		pcl::PointXYZRGB * itP = &cloud->points[0];
        bool is_dense = true;
		
		for(std::size_t y = 0; y < h; ++y){

			const unsigned int offset = y * w;
			const float * itD = itD0 + offset;
			const char * itRGB = itRGB0 + offset * 4;
			const float dy = rowmap(y);

			for(std::size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4 )
			{
				const float depth_value = *itD / 1000.0f;
				
				if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){
	
					const float rx = colmap(x) * depth_value;
                	const float ry = dy * depth_value;               
					itP->z = depth_value;
					itP->x = rx;
					itP->y = ry;

					itP->b = itRGB[0];
					itP->g = itRGB[1];
					itP->r = itRGB[2];
				} else {
					itP->z = qnan_;
					itP->x = qnan_;
					itP->y = qnan_;

					itP->b = qnan_;
					itP->g = qnan_;
					itP->r = qnan_;
					is_dense = false;
 				}
			}
		}
		cloud->is_dense = is_dense;
		listener_.release(frames_);
#ifdef WITH_SERIALIZATION
		if(serialize_)
			serializeCloud(cloud);
#endif
		return cloud;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr updateCloud(const libfreenect2::Frame * rgb, const libfreenect2::Frame * depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
		
		registration_->apply(rgb, depth, &undistorted_, &registered_, true, &big_mat_, map_);

		const std::size_t w = undistorted_.width;
		const std::size_t h = undistorted_.height;

        cv::Mat tmp_itD0(undistorted_.height, undistorted_.width, CV_8UC4, undistorted_.data);
        cv::Mat tmp_itRGB0(registered_.height, registered_.width, CV_8UC4, registered_.data);
        
        if (mirror_ == true){

            cv::flip(tmp_itD0,tmp_itD0,1);
            cv::flip(tmp_itRGB0,tmp_itRGB0,1);

        }

        const float * itD0 = (float *) tmp_itD0.ptr();
        const char * itRGB0 = (char *) tmp_itRGB0.ptr();
        
		pcl::PointXYZRGB * itP = &cloud->points[0];
        bool is_dense = true;
		
		for(std::size_t y = 0; y < h; ++y){

			const unsigned int offset = y * w;
			const float * itD = itD0 + offset;
			const char * itRGB = itRGB0 + offset * 4;
			const float dy = rowmap(y);

			for(std::size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4)
			{
				const float depth_value = *itD / 1000.0f;
				
				if(!std::isnan(depth_value) && !(std::abs(depth_value) < 0.0001)){
	
					const float rx = colmap(x) * depth_value;
                	const float ry = dy * depth_value;               
					itP->z = depth_value;
					itP->x = rx;
					itP->y = ry;

					itP->b = itRGB[0];
					itP->g = itRGB[1];
					itP->r = itRGB[2];
				} else {
					itP->z = qnan_;
					itP->x = qnan_;
					itP->y = qnan_;

					itP->b = qnan_;
					itP->g = qnan_;
					itP->r = qnan_;
					is_dense = false;
 				}
			}
		}
		cloud->is_dense = is_dense;
#ifdef WITH_SERIALIZATION
		if(serialize_)
			serializeCloud(cloud);
#endif
		return cloud;
	}

#endif

	void shutDown(){
		dev_->stop();
  		dev_->close();
	}

	void mirror(){
		mirror_ != mirror_;
	}

	libfreenect2::SyncMultiFrameListener * getListener(){
		return &listener_;
	}

	// Use only if you want only depth, else use get(cv::Mat, cv::Mat) to have the images aligned
	void getDepth(cv::Mat depth_mat){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
		
		cv::Mat depth_tmp(depth->height, depth->width, CV_32FC1, depth->data);
		
		if(mirror_ == true){
        	cv::flip(depth_tmp, depth_mat, 1);
        }else
        {
        	depth_mat = depth_tmp.clone();
        }
		listener_.release(frames_);
	}

	void getIr(cv::Mat ir_mat){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * ir = frames_[libfreenect2::Frame::Ir];
		
		cv::Mat ir_tmp(ir->height, ir->width, CV_32FC1, ir->data);
		
		if(mirror_ == true){
        	cv::flip(ir_tmp, ir_mat, 1);
        }else
        {
        	ir_mat = ir_tmp.clone();
        }
		listener_.release(frames_);
	}

	// Use only if you want only color, else use get(cv::Mat, cv::Mat) to have the images aligned
	void getColor(cv::Mat & color_mat){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];

		cv::Mat tmp_color(rgb->height, rgb->width, CV_8UC4, rgb->data);

		if (mirror_ == true){
			cv::flip(tmp_color, color_mat, 1);
		}else
		{
			color_mat = tmp_color.clone();
		}
		listener_.release(frames_);
	}

	// Depth and color are aligned and registered 
	void get(cv::Mat & color_mat, cv::Mat & depth_mat, const bool full_hd = true, const bool remove_points = false){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

		registration_->apply(rgb, depth, &undistorted_, &registered_, remove_points, &big_mat_, map_);

		cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_32FC1, undistorted_.data);
		cv::Mat tmp_color;
		if(full_hd)
			tmp_color = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
		else
			tmp_color = cv::Mat(registered_.height, registered_.width, CV_8UC4, registered_.data);

		if(mirror_ == true) {
			cv::flip(tmp_depth, depth_mat, 1);
			cv::flip(tmp_color, color_mat, 1);
		}else{
			color_mat = tmp_color.clone();
			depth_mat = tmp_depth.clone();
		}

		listener_.release(frames_);
	}


	// Depth and color are aligned and registered 
	void get(cv::Mat & color_mat, cv::Mat & depth_mat, cv::Mat & ir_mat, const bool full_hd = true, const bool remove_points = false){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
		libfreenect2::Frame * ir = frames_[libfreenect2::Frame::Ir];

		registration_->apply(rgb, depth, &undistorted_, &registered_, remove_points, &big_mat_, map_);

		cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_32FC1, undistorted_.data);
		cv::Mat tmp_color;
		cv::Mat ir_tmp(ir->height, ir->width, CV_32FC1, ir->data);

		if(full_hd)
			tmp_color = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
		else
			tmp_color = cv::Mat(registered_.height, registered_.width, CV_8UC4, registered_.data);

		if(mirror_ == true) {
			cv::flip(tmp_depth, depth_mat, 1);
			cv::flip(tmp_color, color_mat, 1);
			cv::flip(ir_tmp, ir_mat, 1);
		}else{
			color_mat = tmp_color.clone();
			depth_mat = tmp_depth.clone();
			ir_mat = ir_tmp.clone();
		}

		listener_.release(frames_);
	}

#ifdef WITH_PCL
	// All frame and cloud are aligned. There is a small overhead in the double call to registration->apply which has to be removed
	void get(cv::Mat & color_mat, cv::Mat & depth_mat, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, 
		const bool full_hd = true, const bool remove_points = false){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

		registration_->apply(rgb, depth, &undistorted_, &registered_, remove_points, &big_mat_, map_);

		cv::Mat tmp_depth(undistorted_.height, undistorted_.width, CV_32FC1, undistorted_.data);
		cv::Mat tmp_color;

		if(full_hd)
			tmp_color = cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
		else
			tmp_color = cv::Mat(registered_.height, registered_.width, CV_8UC4, registered_.data);

		if (mirror_ == true) {
			cv::flip(tmp_depth, depth_mat, 1);
			cv::flip(tmp_color, color_mat, 1);
		}else{
			color_mat = tmp_color.clone();
			depth_mat = tmp_depth.clone();
		}

		cloud = getCloud(rgb, depth, cloud);
		listener_.release(frames_);
	}
#endif

#ifdef WITH_SERIALIZATION
	void enableSerialization(){
		serialize_ = true;
	}

	void disableSerialization(){
		serialize_ = false;
	}

	bool serialize_status(){
		return serialize_;
	}
#endif

private:

#ifdef WITH_SERIALIZATION
	void serializeFrames(const cv::Mat & depth, const cv::Mat & color)
	{	
		std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();
		unsigned int now = (unsigned int)std::chrono::duration_cast<std::chrono::milliseconds>(tnow.time_since_epoch()).count();
		if(!file_streamer_){
			file_streamer_ = new std::ofstream();
			file_streamer_->open ("stream" + std::to_string(now), std::ios::binary);
			oa_ = new boost::archive::binary_oarchive(*file_streamer_);
		}

		(*oa_) << now << color;
	}
#ifdef WITH_PCL

	void serializeCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
	{	
		std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();
		unsigned int now = (unsigned int)std::chrono::duration_cast<std::chrono::milliseconds>(tnow.time_since_epoch()).count();
		if(!file_streamer_){
			std::cout << "opening stream" << std::endl;
			file_streamer_ = new std::ofstream();
			file_streamer_->open ("stream" + std::to_string(now), std::ios::binary);
		}

		microser sr(*file_streamer_);
		sr << now << (uint32_t)cloud->size();
		for(auto &p : cloud->points){
			sr << p.x << p.y << p.z << p.r << p.g << p.b;
		}
	}
#endif
#endif

	void prepareMake3D(const libfreenect2::Freenect2Device::IrCameraParams & depth_p)
	{
		const int w = 512;
		const int h = 424;
	    float * pm1 = colmap.data();
	    float * pm2 = rowmap.data();
	    for(int i = 0; i < w; i++)
	    {
	        *pm1++ = (i-depth_p.cx + 0.5) / depth_p.fx;
	    }
	    for (int i = 0; i < h; i++)
	    {
	        *pm2++ = (i-depth_p.cy + 0.5) / depth_p.fy;
	    }
	}

	libfreenect2::Freenect2 freenect2_;
	libfreenect2::Freenect2Device * dev_ = 0;
	libfreenect2::PacketPipeline * pipeline_ = 0;
	libfreenect2::Registration * registration_ = 0;
	libfreenect2::SyncMultiFrameListener listener_;
	libfreenect2::Logger * logger_ = nullptr;
	libfreenect2::FrameMap frames_;
	libfreenect2::Frame undistorted_, registered_, big_mat_;
	Eigen::Matrix<float,512,1> colmap;
	Eigen::Matrix<float,424,1> rowmap;
	std::string serial_;
	int map_[512 * 424]; 
	float qnan_; 
	bool mirror_;
#ifdef WITH_SERIALIZATION
	bool serialize_;
	std::ofstream * file_streamer_;
	boost::archive::binary_oarchive * oa_;
#endif  
};