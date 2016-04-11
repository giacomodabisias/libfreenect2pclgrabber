#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
#include "k2g.h"
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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"

class Kinect2Grabber;

struct PlySaver{

  PlySaver(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, bool binary, bool use_camera, Kinect2Grabber & ros_grabber): 
           cloud_(cloud), binary_(binary), use_camera_(use_camera), K2G_ros_(ros_grabber){}

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
  bool binary_;
  bool use_camera_;
  Kinect2Grabber & K2G_ros_;

};

void createImage(cv::Mat & image, std_msgs::Header & header, sensor_msgs::Image & msgImage, const bool color) const
{
	size_t step, size;
	step = image.cols * image.elemSize();
	size = image.rows * step;
	if(color)
		msgImage.encoding = sensor_msgs::image_encodings::TYPE_32UC1;
	else
		msgImage.encoding = sensor_msgs::image_encodings::TYPE_32UC3;

	msgImage.header = header;
	msgImage.height = image.rows;
	msgImage.width = image.cols;
	msgImage.is_bigendian = false;
	msgImage.step = step;
	msgImage.data.resize(size);

	memcpy(msgImage.data.data(), image.data, size);
}


class Kinect2Grabber 
{

public:

	Kinect2Grabber(processor freenect_processor = CPU): k2g_(freenect_processor), shutdown_(false), 
		header_color_.frame_id("kinect2_rgb_optical_frame"), 
		header_depth_.frame_id("kinect2_depth_optical_frame"),
		header_cloud_.frame_id("kinect2_rgb_optical_frame"),
		point_cloud_2_.header.frame_id("kinect2_rgb_optical_frame");
	{
		point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/kinect2/hd/points", 1);
		color_pub_ = nh_.advertise<sensor_msgs::Image>("/kinect2/hd/image_color", 1);
		camera_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/kinect2/hd/camera_info", 1);
		depth_pub_ = nh_.advertise<sensor_msgs::Image>("/kinect2/sd/image_depth", 1);

		libfreenect2::Freenect2Device::IrCameraParams ir = k2g_.getIrParameters();
		libfreenect2::Freenect2Device::ColorCameraParams rgb = k2g_.getRgbParameters();
	}


	// Use only if you want only color, else use get(cv::Mat, cv::Mat) to have the images aligned
	void publishDepth(){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * depth = frames_[libfreenect2::Frame::Depth];
		
		cv::Mat tmp_depth(depth->height, depth->width, CV_32FC1, depth->data);
		
		if(mirror_ == true)
        	cv::flip(tmp_depth, tmp_depth, 1);

		header_depth_.stamp = ros::Time::now();
		
		createImage(tmp_depth, header_depth_, depth_image_, false);
		depth_pub_.publish(depth_image_);

		listener_.release(frames_);
	}

	// Use only if you want only color, else use get(cv::Mat, cv::Mat) to have the images aligned
	void publishColor(){
		listener_.waitForNewFrame(frames_);
		libfreenect2::Frame * rgb = frames_[libfreenect2::Frame::Color];

		cv::Mat tmp_color(rgb->height, rgb->width, CV_8UC4, rgb->data);

		if (mirror_ == true){
			cv::flip(tmp_color, color_mat, 1);

		header_color_.stamp = ros::Time::now();
		color_image_ = cv_bridge::CvImage(header_color_, "bgra8", tmp_color).toImageMsg();
		color_pub_.publish(color_image_);

		listener_.release(frames_);
	}

	// Depth and color are aligned and registered 
	void publishDepthColor(const bool full_hd = true, const bool remove_points = false){
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
			cv::flip(tmp_depth, tmp_depth, 1);
			cv::flip(tmp_color, tmp_color, 1);
		}

		header_depth_.stamp = ros::Time::now();
		
		createImage(tmp_depth, header_depth_, depth_image_, false);
		depth_pub_.publish(depth_image_);

		header_color_.stamp = ros::Time::now();
		
		color_image_ = cv_bridge::CvImage(header_color_, "bgra8", tmp_color).toImageMsg();
		color_pub_.publish(color_image_);

		listener_.release(frames_);
	}

	// All frame and cloud are aligned. There is a small overhead in the double call to registration->apply which has to be removed
	void publishAll(const bool full_hd = true, const bool remove_points = false){

		cv::Mat tmp_depth, tmp_color;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

		get(tmp_color, tmp_depth, cloud, full_hd, remove_points);

		header_depth_.stamp = ros::Time::now();
		
		createImage(tmp_depth, header_depth_, depth_image_, false);
		depth_pub_.publish(depth_image_);

		header_color_.stamp = ros::Time::now();
		
		color_image_ = cv_bridge::CvImage(header_color_, "bgra8", tmp_color).toImageMsg();
		color_pub_.publish(color_image_);

		pcl::toROSMsg(*cloud_, point_cloud_2_);
		
		point_cloud_2_.header.stamp = ros::Time::now();
		point_cloud_pub_.publish(point_cloud_2_);
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
    std_msgs::Header header_depth_, header_color_, header_cloud_;
    sensor_msgs::Image depth_image_;
    sensor_msgs::Image::Ptr color_image_;
    K2G k2g_;
    bool shutdown_;
};
