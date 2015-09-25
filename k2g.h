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
Giacomo. Dabisias, PhD Student
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/registration.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <vector>
#include <signal.h>
#include <string>
#include <iostream>
#include <Eigen/Core>

bool stop = false;

enum processor{
	CPU, OPENCL, OPENGL
};

void sigint_handler(int s)
{
	stop = true;
}

class K2G {

public:

	K2G(processor p): undistorted_(512, 424, 4), registered_(512, 424, 4), listener_(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth){

		signal(SIGINT,sigint_handler);

		if(freenect2_.enumerateDevices() == 0)
		{
			std::cout << "no kinect2 connected!" << std::endl;
			exit(-1);
		}

		serial_ = freenect2_.getDefaultDeviceSerialNumber();
		switch(p){
			case CPU:
				std::cout << "creating CPU processor" << std::endl;
				pipeline_ = new libfreenect2::CpuPacketPipeline();
				break;
			case OPENCL:
				std::cout << "creating OpenCL processor" << std::endl;
				pipeline_ = new libfreenect2::OpenCLPacketPipeline();
				break;
			case OPENGL:
				std::cout << "creating OpenGL processor" << std::endl;
				pipeline_ = new libfreenect2::OpenGLPacketPipeline();
				break;
			default:
				std::cout << "creating CPU processor" << std::endl;
				pipeline_ = new libfreenect2::CpuPacketPipeline();
				break;
		}
		
		dev_ = freenect2_.openDevice(serial_, pipeline_);
		dev_->setColorFrameListener(&listener_);
		dev_->setIrAndDepthFrameListener(&listener_);
		dev_->start();

		registration_ = new libfreenect2::Registration(dev_->getIrCameraParams(), dev_->getColorCameraParams());

		d_matrix_ = Eigen::Matrix4d::Identity();
		d_matrix_(0,0) = dev_->getIrCameraParams().fx;
		d_matrix_(0,2) = dev_->getIrCameraParams().cx;
		d_matrix_(1,1) = dev_->getIrCameraParams().fy;
		d_matrix_(1,2) = dev_->getIrCameraParams().cy;

		d_matrix_inv_ = d_matrix_.inverse();
		std::cout << "Camera matrix:" << std::endl;
		std::cout << d_matrix_ << std::endl;
 	}

 	/*
	void addCallback(std::function f){
		callbacks_.push_back(f);
	}*/

	libfreenect2::Freenect2Device::IrCameraParams getIrParameters(){
		libfreenect2::Freenect2Device::IrCameraParams ir = dev_->getIrCameraParams();
		return ir;
	}

	libfreenect2::Freenect2Device::ColorCameraParams getRgbParameters(){
		libfreenect2::Freenect2Device::ColorCameraParams rgb = dev_->getColorCameraParams();
		return rgb;
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){
		
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];

		registration_->apply(rgb, depth, &undistorted_, &registered_);
		const short w = undistorted_.width;
		const short h = undistorted_.height;

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(w, h));

		const float * itD0 = (float *)undistorted_.data;
		const char * itRGB0 = (char *)registered_.data;
		pcl::PointXYZRGB * itP = &cloud->points[0];
		
		for(int y = 0; y < h; ++y)
		{	
			const unsigned int offset = y * w;
			const float * itD = itD0 + offset;
			const char * itRGB = itRGB0 + offset*4;
			
			for(size_t x = 0; x < w; ++x, ++itP, ++itD, itRGB += 4 )
			{
				const float depth_value = *itD / 1000.0f;
				
				if(isnan(depth_value))
				{
					itP->x = itP->y = itP->z = std::numeric_limits<float>::quiet_NaN();
					itP->rgba = 0;

				}else{

					Eigen::Vector4d psd(x, y, 1.0, 1.0 / depth_value);
					pworld_ = d_matrix_inv_ * psd * depth_value;
					itP->z = depth_value;

					itP->x = isnan(pworld_.x()) ? 0 : pworld_.x();
					itP->y = isnan(pworld_.y()) ? 0 : pworld_.y();

					itP->b = itRGB[0];
					itP->g = itRGB[1];
					itP->r = itRGB[2];
				}
			}
		}

		listener_.release(frames_);
		return cloud;
	}

	void shutDown(){
		dev_->stop();
  		dev_->close();
	}

	cv::Mat * getColor(){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];
		cv::Mat * tmp = new cv::Mat(rgb->height, rgb->width, CV_8UC4, rgb->data);
		listener_.release(frames_);
		return tmp;
	}
private:

	libfreenect2::Freenect2 freenect2_;
	libfreenect2::Freenect2Device * dev_ = 0;
	libfreenect2::PacketPipeline * pipeline_ = 0;
	libfreenect2::Registration * registration_ = 0;
	libfreenect2::SyncMultiFrameListener listener_;
	libfreenect2::FrameMap frames_;
	libfreenect2::Frame undistorted_, registered_;
	Eigen::Vector4d pworld_;
	Eigen::Matrix4d d_matrix_, d_matrix_inv_;
	//std::vector<std::function> callbacks_;
	std::string serial_;
};