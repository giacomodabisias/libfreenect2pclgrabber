/*
Copyright 2015, Giacomo Dabisias"
Copyright 2016, Michele Mambrini"
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
Giacomo  Dabisias, PhD Student
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
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <ctime>
#include <time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>



struct PlySaver{

  PlySaver(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, bool binary, bool use_camera, Kinect2Grab & ros_grabber): 
           cloud_(cloud), binary_(binary), use_camera_(use_camera), K2G_ros_(ros_grabber){}

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
  bool binary_;
  bool use_camera_;
  Kinect2Grab & K2G_ros_;

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
		if(pressed == "q")
		{
			K2G_ros_.shutDown();
		}
	}
}


class Kinect2Grab 
{

public:

	Kinect2Grab(processor freenect_processor = CPU): k2g_(freenectprocessor), shutdown_(false)
	{
		  
		pointCloudPub_ = nh_.advertise<sensor_msgs::PointCloud2>("/kinect2/hd/points", 1);
		colorImagePub_ = nh_.advertise<sensor_msgs::Image>("/kinect2/hd/image_color", 1);
		camera_info_Pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/kinect2/hd/camera_info", 1);

		libfreenect2::Freenect2Device::IrCameraParams ir =  k2g_.getIrParameters();
		libfreenect2::Freenect2Device::ColorCameraParams = k2g_.getRgbParameters();

	}

	void publishAll()
	{
		std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();

		if(cloud_)
			cloud_ = k2g_->updateCloud(cloud_);
		else
			cloud_ = k2g_->get(cloud_);

		color_ = k2g_->getColor();
		std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
		
		pcl::toROSMsg( *cloud_, point_cloud_2_);
		point_cloud_2_.header.frame_id="/base_link";
		point_cloud_2_.header.stamp = ros::Time::now();
		
		sensor_msgs::Image::Ptr color_image_ = cv_bridge::CvImage(head_, "bgra8", color_).toImageMsg();

		colorImagePub_.publish(color_image_);
		pointCloudPub_.publish(point_cloud_2_);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){
		return k2g_->getCloud();
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
		return k2g_->updateCloud();
	}

	void shutDown(){
		k2g_.shutDown();
	}
    
private:
	ros::NodeHandle nh_;
	ros::Publisher pointCloudPub_, colorImagePub_, camera_info_Pub_;
    sensor_msgs::PointCloud2 point_cloud_2_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
    cv::Mat color_;
    std_msgs::Header head_;
    K2G k2g_;
    bool shutdown_;
};


int main(int argc, char *argv[])
{
  
	ros::init (argc, argv, "RosKinect2Grabber");
	ros::NodeHandle nh;
	Kinect2Grab K2G_ros;

	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
	cloud = k2g_->getCloud();

	cloud->sensor_orientation_.w() = 0.0;
	cloud->sensor_orientation_.x() = 1.0;
	cloud->sensor_orientation_.y() = 0.0;
	cloud->sensor_orientation_.z() = 0.0; 

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

	PlySaver ps(cloud, false, false, K2G_ros);
	viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);	

	while((ros::ok()) && (!K2G_ros->terminate()))
	{  		
		viewer_->spinOnce ();
		cloud = updateCloud(cloud);
		K2G_ros.publishAll();  
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");     	 
	}

	K2G_ros.shutDown();
	return 0;
}

