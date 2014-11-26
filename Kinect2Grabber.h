#include <libfreenect2/opengl.h>
#include <signal.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/rgb_packet_stream_parser.h>
#include <depth_registration.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "depth_registration_cpu.h"


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define DEPTH_REG_CPU
bool shutdown = false;  //bad

void sigint_handler(int s)
{
  shutdown = true;
}

namespace Kinect2 {

template< typename PointT>
class Kinect2Grabber
{
public:

 

	Kinect2Grabber(int mode): mode_(mode), cloud_(new pcl::PointCloud<PointT>()) {
	glfwInit();
		dev_ = freenect2_.openDefaultDevice();
		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
		if(dev_ == 0){
			std::cout << "no device connected or failure opening the default one!" << std::endl;
			exit(1);
		}
		signal(SIGINT,sigint_handler);
		shutdown = false;
		dev_->setColorFrameListener(listener_);
		dev_->setIrAndDepthFrameListener(listener_);
		dev_->start();

		color_camera_params_  = dev_->getColorCameraParams();
		ir_camera_params_ = dev_->getIrCameraParams();
		std::cout << "color:" << std::endl;
		std::cout <<"\tfx " << color_camera_params_.fx <<std::endl;
		std::cout <<"\tfy " << color_camera_params_.fy <<std::endl;
		std::cout <<"\tcx " << color_camera_params_.cx <<std::endl;
		std::cout <<"\tcy " << color_camera_params_.cy <<std::endl;
		std::cout <<"ir " << std::endl;
		std::cout <<"\tfx " << ir_camera_params_.fx <<std::endl;
		std::cout <<"\tfy " << ir_camera_params_.fy <<std::endl;
		std::cout <<"\tcx " << ir_camera_params_.cx <<std::endl;
		std::cout <<"\tcy " << ir_camera_params_.cy <<std::endl;

		std::cout << "device initialized" << std::endl;
		std::cout << "device serial: " << dev_->getSerialNumber() << std::endl;
		std::cout << "device firmware: " << dev_->getFirmwareVersion() << std::endl;

	}


	~Kinect2Grabber(){
		this->ShutDown();
	}

	libfreenect2::Frame *
	GetRgbFrame(){
		listener_->waitForNewFrame(frames_);
		return frames_[libfreenect2::Frame::Color];
	}

	libfreenect2::Frame *
	GetIrFrame(){
		listener_->waitForNewFrame(frames_);
		return frames_[libfreenect2::Frame::Ir];
	} 

	libfreenect2::Frame *
	GetDepthFrame(){
		listener_->waitForNewFrame(frames_);
		return frames_[libfreenect2::Frame::Depth];
	}

	libfreenect2::FrameMap *
	GetRawFrames(){
	listener_->waitForNewFrame(frames_);
	return &frames_;
	}

	void
	ShutDown(){
		dev_->stop();
		dev_->close();
	}

	libfreenect2::Freenect2Device::ColorCameraParams 
	GetColorParams(){
		return color_camera_params_;
	}

	libfreenect2::Freenect2Device::IrCameraParams 
	GetIrPArams(){
		return ir_camera_params_;
	}

	void
	FreeFrames(){
		listener_->release(frames_);
	}

	
	void createLookup()
	{
		const double fx = 1.0 / cameraMatrixColor_.at<double>(0, 0);
		const double fy = 1.0 / cameraMatrixColor_.at<double>(1, 1);
		const double cx = cameraMatrixColor_.at<double>(0, 2);
		const double cy = cameraMatrixColor_.at<double>(1, 2);
		double *it;

		lookupY_ = cv::Mat(1, size_registered_.height, CV_64F);
		it = lookupY_.ptr<double>();
		for(size_t r = 0; r < (size_t)size_registered_.height; ++r, ++it)
		{
		*it = (r - cy) * fy;
		}

		lookupX_ = cv::Mat(1, size_registered_.width, CV_64F);
		it = lookupX_.ptr<double>();
		for(size_t c = 0; c < (size_t)size_registered_.width; ++c, ++it)
		{
		*it = (c - cx) * fx;
		}
	}


	void createCloud(const cv::Mat &depth, const cv::Mat &color, typename pcl::PointCloud<PointT>::Ptr &cloud) const
	{
		const float badPoint = std::numeric_limits<float>::quiet_NaN();

		#pragma omp parallel for
		for(int r = 0; r < depth.rows; ++r)
		{
		  PointT *itP = &cloud->points[r * depth.cols];
		  const uint16_t *itD = depth.ptr<uint16_t>(r);
		  const cv::Vec3b *itC = color.ptr<cv::Vec3b>(r);

		  for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC )
		  {
		    register const float depthValue = *itD / 1000.0f;
		    // Check for invalid measurements

			if(isnan(depthValue) || depthValue <= 0.001)
			{
			   // not valid
			   itP->x = itP->y = itP->z = badPoint;
			   itP->rgba = 0;
			   continue;
			}
			itP->z = depthValue;
			itP->x = ((c - color_camera_params_.cx) / color_camera_params_.fx) * depthValue;
			itP->y = ((r - color_camera_params_.cy) / color_camera_params_.fy) * depthValue;
			itP->b = itC->val[0];
			itP->g = itC->val[1];
			itP->r = itC->val[2];
		  }
		}
	}


  typename pcl::PointCloud<PointT>::Ptr
  GetCloud(){
	
	frames_ =  *GetRawFrames();
	rgb_ = frames_[libfreenect2::Frame::Color];
	depth_ = frames_[libfreenect2::Frame::Depth];
	tmp_depth_ = cv::Mat(depth_->height, depth_->width, CV_32FC1, depth_->data);
	tmp_rgb_ = cv::Mat(rgb_->height, rgb_->width, CV_8UC3, rgb_->data);
	cv::flip(tmp_depth_, tmp_depth_, 1);
	cv::flip(tmp_rgb_, tmp_rgb_, 1);
	tmp_depth_.convertTo(tmp_depth_, CV_16U);

	if(depthReg_ == 0){

		cameraMatrixColor_ = (cv::Mat_<double>(3,3) <<  
			color_camera_params_.fx,                0.,               color_camera_params_.cx,
					0.,                  color_camera_params_.fy,     color_camera_params_.cy,
			        0.,                             0.,                         1.             );

		cameraMatrixDepth_ = (cv::Mat_<double>(3,3) <<
			ir_camera_params_.fx,                   0.,               ir_camera_params_.cx,
			        0.,                  ir_camera_params_.fy,        ir_camera_params_.cy,
			        0.,                             0.,                         1.             );

		rotation_ = (cv::Mat_<double>(3,3) <<
			9.9983695759005575e-01, -1.6205694274052811e-02,-7.9645282444769407e-03, 
			1.6266694212988934e-02, 9.9983838631845712e-01, 7.6547961099503467e-03,
       		7.8391897822575607e-03, -7.7831045990485416e-03, 9.9993898333166209e-01  );

		translation_ = (cv::Mat_<double>(3,1) <<
			-5.1927840124258939e-02, -4.5307585220976776e-04, 7.0571985343338605e-05 );

		distorsion_ = (cv::Mat_<double>(1,5) <<
			ir_camera_params_.k1,    ir_camera_params_.k2, 1.0698071918753883e-03,
			1.1438966542906426e-04,  ir_camera_params_.k3                          );  			
		
		depthReg_ = new DepthRegistrationCPU();

		size_registered_ = cv::Size(1920, 1080); 
		size_depth_ = cv::Size(512, 424);


		depthReg_->init(cameraMatrixColor_,
						size_registered_,
					    cameraMatrixDepth_,
					    size_depth_,
					    distorsion_,
						rotation_, 
						translation_ );

		cloud_->height = size_registered_.height;
		cloud_->width = size_registered_.width;
		cloud_->is_dense = false;
		cloud_->points.resize(cloud_->height * cloud_->width);
		

	}
	
	depthReg_->registerDepth(tmp_depth_, registered_ );
	createCloud(registered_, tmp_rgb_, cloud_);
	
	return cloud_;
  }

private:

	libfreenect2::Freenect2 freenect2_;
	libfreenect2::Freenect2Device * dev_ = 0;
	libfreenect2::SyncMultiFrameListener * listener_ = 0;
	libfreenect2::FrameMap frames_;
	DepthRegistration * depthReg_ = 0;
	libfreenect2::Freenect2Device::ColorCameraParams color_camera_params_;
	libfreenect2::Freenect2Device::IrCameraParams ir_camera_params_;
	int mode_;
	cv::Mat scaled_, registered_, translation_, rotation_;
	libfreenect2::Frame *depth_;
	libfreenect2::Frame *rgb_;
	cv::Size size_registered_, size_depth_;
	cv::Mat lookupY_, lookupX_, tmp_depth_, tmp_rgb_, cameraMatrixColor_, cameraMatrixDepth_, distorsion_;
	typename pcl::PointCloud<PointT>::Ptr cloud_;

};

}