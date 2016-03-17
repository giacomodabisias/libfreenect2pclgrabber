/*
Copyright 2016, Giacomo Dabisias & Michele Mambrini"
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
Giacomo  Dabisias, PhD Student & Michele Mambrini
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/
#include "k2g.h"
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <ctime>
#include <time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"


class Kinect2Grab;

struct PlySaver{

  PlySaver(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, bool binary, bool use_camera, Kinect2Grab & ros_grabber): 
           cloud_(cloud), binary_(binary), use_camera_(use_camera), K2G_ros_(ros_grabber){}

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
  bool binary_;
  bool use_camera_;
  Kinect2Grab & K2G_ros_;

};

void createImage(cv::Mat &image, std_msgs::Header &header, sensor_msgs::Image &msgImage) 
{
	size_t step, size;
	step = image.cols * image.elemSize();
	size = image.rows * step;
	msgImage.encoding = sensor_msgs::image_encodings::TYPE_16UC1;

	msgImage.header = header;
	msgImage.height = image.rows;
	msgImage.width = image.cols;
	msgImage.is_bigendian = false;
	msgImage.step = step;
	msgImage.data.resize(size);

	memcpy(msgImage.data.data(), image.data, size);
}


class Kinect2Grab 
{

public:

	Kinect2Grab(processor freenect_processor = CPU): k2g_(freenect_processor), shutdown_(false)
	{
		point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/kinect2/hd/points", 1);
		color_pub_ = nh_.advertise<sensor_msgs::Image>("/kinect2/hd/image_color", 1);
		camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/kinect2/hd/camera_info", 1);
		depth_pub_ = nh_.advertise<sensor_msgs::Image>("/kinect2/sd/image_depth", 1);

		libfreenect2::Freenect2Device::IrCameraParams ir = k2g_.getIrParameters();
		libfreenect2::Freenect2Device::ColorCameraParams rgb = k2g_.getRgbParameters();
	}

	void publishAll()
	{
		std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();

		if(cloud_)
			cloud_ = k2g_.updateCloud(cloud_);
		else
			cloud_ = k2g_.getCloud();

		depth_ = k2g_.getDepth();
		color_ = k2g_.getColor();
		std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
		
		pcl::toROSMsg( *cloud_, point_cloud_2_);
		point_cloud_2_.header.frame_id="world";
		point_cloud_2_.header.stamp = ros::Time::now();
		
		sensor_msgs::Image::Ptr color_image_ = cv_bridge::CvImage(head_, "bgra8", color_).toImageMsg();
		depth_.convertTo(depth_, CV_16U);
		std_msgs::Header header;
		header.frame_id = "kinect2_depth";
		header.stamp = ros::Time::now();
		sensor_msgs::Image depth_image_;

		createImage(depth_, header, depth_image_);

		color_pub_.publish(color_image_);
		point_cloud_pub_.publish(point_cloud_2_);
		depth_pub_.publish(depth_image_);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){
		return k2g_.getCloud();
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
		return k2g_.updateCloud(cloud);
	}

	void shutDown(){
		k2g_.shutDown();
	}

	bool terminate(){
		return shutdown_;
	}

	void setShutdown(){
		shutdown_ = true;
	}
    
private:
	ros::NodeHandle nh_;
	ros::Publisher point_cloud_pub_, color_pub_, camera_info_pub_, depth_pub_;
    sensor_msgs::PointCloud2 point_cloud_2_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
    cv::Mat color_, depth_;
    std_msgs::Header head_;
    K2G k2g_;
    bool shutdown_;
};

void
KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data)
{
	std::string pressed;
	pressed = event.getKeySym();
	PlySaver * s = (PlySaver*)data;
	if(event.keyDown ())
	{
		if(pressed == "s")
		{
		  
			pcl::PLYWriter writer;
			std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
			std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
			writer.write ("cloud_" + now, *(s->cloud_), s->binary_, s->use_camera_);
			std::cout << "saved " << "cloud_" + now + ".ply" << std::endl;
		}
		if(pressed == "e")
		{
			s->K2G_ros_.setShutdown();
			std::cout << "SHUTTING DOWN" << std::endl;
		}
	}
}


int main(int argc, char *argv[])
{
	std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively\n";
	processor freenectprocessor = OPENGL;

	if(argc > 1){
		freenectprocessor = static_cast<processor>(atoi(argv[1]));
	}
	ros::init (argc, argv, "RosKinect2Grabber");

	Kinect2Grab K2G_ros(freenectprocessor);

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
	cloud = K2G_ros.getCloud();

	cloud->sensor_orientation_.w() = 0.0;
	cloud->sensor_orientation_.x() = 1.0;
	cloud->sensor_orientation_.y() = 0.0;
	cloud->sensor_orientation_.z() = 0.0; 
/*
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	PlySaver ps(cloud, false, false, K2G_ros);
	viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);	
*/
	while((ros::ok()) && (!K2G_ros.terminate()))
	{  		
		//viewer->spinOnce ();
		cloud = K2G_ros.updateCloud(cloud);
		K2G_ros.publishAll();  
		//pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		//viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");     	 
	}

	K2G_ros.shutDown();
	return 0;
}

