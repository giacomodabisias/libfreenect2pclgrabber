#include <chrono>
#include "k2g.h"
#include <pcl/pcl_base.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class K2GRos 
{

public:

	K2GRos(Processor freenect_processor = CPU): k2g_(freenect_processor), 
			cloud_(new pcl::PointCloud<pcl::PointXYZRGB>(512, 424)),
			size_color_(1920, 1080),
		  	size_depth_(512, 424)
	{
		header_color_.frame_id = "kinect2_rgb_optical_frame";
		header_depth_.frame_id = "kinect2_ir_optical_frame";
		header_cloud_.frame_id = "kinect2_rgb_optical_frame";
		
		point_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/kinect2/hd/points", 1);
		color_pub_ = nh_.advertise<sensor_msgs::Image>("/kinect2/hd/image_color", 1);
		color_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/kinect2/hd/camera_info", 1);
		depth_pub_ = nh_.advertise<sensor_msgs::Image>("/kinect2/sd/image_depth", 1);
		depth_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/kinect2/sd/camera_info", 1);

		libfreenect2::Freenect2Device::IrCameraParams ir = k2g_.getIrParameters();
		libfreenect2::Freenect2Device::ColorCameraParams rgb = k2g_.getRgbParameters();

		createCameraInfoColor(rgb);
		createCameraInfoDepth(ir);
	}

	// Use only if you want only color, else use get(cv::Mat, cv::Mat) to have the images aligned
	void publishDepth(){
		cv::Mat tmp_depth;
		
		k2g_.getDepth(tmp_depth);
		header_depth_.stamp = ros::Time::now();
		
		createImage(tmp_depth, header_depth_, depth_image_, false);
		depth_pub_.publish(depth_image_);
	}

	// Use only if you want only color, else use get(cv::Mat, cv::Mat) to have the images aligned
	void publishColor(){

		cv::Mat tmp_color;
		k2g_.getColor(tmp_color);

		header_color_.stamp = ros::Time::now();
		color_image_ = cv_bridge::CvImage(header_color_, "bgra8", tmp_color).toImageMsg();
		color_pub_.publish(color_image_);
	}

	// Depth and color are aligned and registered 
	void publishDepthColor(const bool full_hd = true, const bool remove_points = false){
		
		cv::Mat tmp_depth, tmp_color;
		k2g_.get(tmp_depth, tmp_color, full_hd, remove_points);
		
		header_depth_.stamp = ros::Time::now();
		
		createImage(tmp_depth, header_depth_, depth_image_, false);
		depth_pub_.publish(depth_image_);

		header_color_.stamp = ros::Time::now();
		
		color_image_ = cv_bridge::CvImage(header_color_, "bgra8", tmp_color).toImageMsg();
		color_pub_.publish(color_image_);

	}

	// All frame and cloud are aligned. There is a small overhead in the double call to registration->apply which has to be removed
	void publishAll(const bool full_hd = true, const bool remove_points = false){

		cv::Mat tmp_depth, tmp_color;

		k2g_.get(tmp_color, tmp_depth, cloud_, full_hd, remove_points);

		header_depth_.stamp = ros::Time::now();
		
		createImage(tmp_depth, header_depth_, depth_image_, false);
		depth_pub_.publish(depth_image_);

		header_color_.stamp = ros::Time::now();
			
		color_image_ = cv_bridge::CvImage(header_color_, "bgra8", tmp_color).toImageMsg();
		color_pub_.publish(color_image_);
		pcl::toROSMsg(*cloud_, point_cloud_2_);

		point_cloud_2_.header.frame_id = "world";

		point_cloud_2_.header.stamp = ros::Time::now();
		point_cloud_pub_.publish(point_cloud_2_);
	}

	void shutDown(){
		k2g_.shutDown();
	}

	bool terminate(){
		return stop;
	}

	void setShutdown(){
		stop = true;
	}
	
	void publishCameraInfoColor(){
		color_info_pub_.publish(camera_info_color_);
	}

	void publishCameraInfoDepth(){
		depth_info_pub_.publish(camera_info_depth_);
	}

	void mirror(){
		k2g_.mirror();
	}
   
private:

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr getCloud(){
		return k2g_.getCloud();
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr updateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud){
		return k2g_.updateCloud(cloud);
	}

	void createImage(cv::Mat & image, std_msgs::Header & header, sensor_msgs::Image & msgImage, const bool color) const
	{	
		size_t step, size;
		step = image.cols * image.elemSize();
		size = image.rows * step;
		if(color)
			msgImage.encoding = sensor_msgs::image_encodings::BGRA8;
		else
			msgImage.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

		msgImage.header = header;
		msgImage.height = image.rows;
		msgImage.width = image.cols;
		msgImage.is_bigendian = false;
		msgImage.step = step;
		msgImage.data.resize(size);

		memcpy(msgImage.data.data(), image.data, size);
	}


	void createCameraInfoColor(libfreenect2::Freenect2Device::ColorCameraParams color_params)
	{
	  cv::Mat proj_matrix_color = cv::Mat::zeros(3, 4, CV_64F);
	  cv::Mat camera_matrix_color = cv::Mat::eye(3, 3, CV_64F);
	  cv::Mat distortion_matrix_color = cv::Mat::zeros(1, 5, CV_64F);

	  camera_matrix_color.at<double>(0, 0) = color_params.fx;
	  camera_matrix_color.at<double>(1, 1) = color_params.fy;
	  camera_matrix_color.at<double>(0, 2) = color_params.cx;
	  camera_matrix_color.at<double>(1, 2) = color_params.cy;
	  camera_matrix_color.at<double>(2, 2) = 1;
	  camera_matrix_color.copyTo(proj_matrix_color(cv::Rect(0, 0, 3, 3)));
	  
	  createCameraInfo(size_color_, camera_matrix_color, distortion_matrix_color, cv::Mat::eye(3, 3, CV_64F), 
	  																   proj_matrix_color, camera_info_color_, true);
	}

	void createCameraInfoDepth(libfreenect2::Freenect2Device::IrCameraParams ir_params)
	{
		cv::Mat proj_matrix_depth = cv::Mat::zeros(3, 4, CV_64F);
		cv::Mat camera_matrix_depth = cv::Mat::eye(3, 3, CV_64F);
		cv::Mat distortion_matrix_depth = cv::Mat::zeros(1, 5, CV_64F);

		camera_matrix_depth.at<double>(0, 0) = ir_params.fx;
		camera_matrix_depth.at<double>(1, 1) = ir_params.fy;
		camera_matrix_depth.at<double>(0, 2) = ir_params.cx;
		camera_matrix_depth.at<double>(1, 2) = ir_params.cy;
		camera_matrix_depth.at<double>(2, 2) = 1;
		camera_matrix_depth.copyTo(proj_matrix_depth(cv::Rect(0, 0, 3, 3)));
		
		createCameraInfo(size_depth_, camera_matrix_depth, distortion_matrix_depth, cv::Mat::eye(3, 3, CV_64F), 
																		   proj_matrix_depth, camera_info_depth_, false);
	}

	void createCameraInfo(const cv::Size &size, const cv::Mat &cameraMatrix, const cv::Mat &distortion, const cv::Mat &rotation, 
						  const cv::Mat &projection, sensor_msgs::CameraInfo &cameraInfo, const bool color ) const
	{

		if (color)
		{
			cameraInfo.header.frame_id = "kinect2_rgb_optical_frame";	
		}
		else
		{
			cameraInfo.header.frame_id = "kinect2_ir_optical_frame";	
		}
		cameraInfo.height = size.height;
		cameraInfo.width = size.width;

		const double *itC = cameraMatrix.ptr<double>(0, 0);
		for(size_t i = 0; i < 9; ++i, ++itC)
		{
			cameraInfo.K[i] = *itC;
		}

		const double *itR = rotation.ptr<double>(0, 0);
		for(size_t i = 0; i < 9; ++i, ++itR)
		{
			cameraInfo.R[i] = *itR;
		}

		const double *itP = projection.ptr<double>(0, 0);
		for(size_t i = 0; i < 12; ++i, ++itP)
		{
			cameraInfo.P[i] = *itP;
		}

		cameraInfo.distortion_model = "plumb_bob";
		cameraInfo.D.resize(distortion.cols);
		const double *itD = distortion.ptr<double>(0, 0);
		
		for(size_t i = 0; i < (size_t)distortion.cols; ++i, ++itD)
		{
			cameraInfo.D[i] = *itD;
		}
	}

	ros::NodeHandle nh_;
	ros::Publisher point_cloud_pub_, color_pub_, color_info_pub_, depth_pub_, depth_info_pub_;
    sensor_msgs::PointCloud2 point_cloud_2_;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
    cv::Mat color_, depth_;
    cv::Size size_color_, size_depth_;
    std_msgs::Header header_depth_, header_color_, header_cloud_;
    sensor_msgs::Image depth_image_;
    sensor_msgs::Image::Ptr color_image_;
    K2G k2g_;
    libfreenect2::SyncMultiFrameListener * listener_;
    sensor_msgs::CameraInfo  camera_info_color_, camera_info_depth_;

};
