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

 

	Kinect2Grabber(int mode): mode_(mode) {
	glfwInit();
		dev_ = freenect2_.openDefaultDevice();
		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
		if(dev_ == 0)
		{
		std::cout << "no device connected or failure opening the default one!" << std::endl;
		exit(1);
		}
		signal(SIGINT,sigint_handler);
		shutdown = false;
		dev_->setColorFrameListener(listener_);
		dev_->setIrAndDepthFrameListener(listener_);
		dev_->start();

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
		return dev_->getColorCameraParams();
	}

	libfreenect2::Freenect2Device::IrCameraParams 
	GetIrPArams(){
		return dev_->getIrCameraParams();
	}

	void
	FreeFrames(){
		listener_->release(frames_);
	}

	void createLookup(size_t width, size_t height) //READ VALUES FROM MATRIX
	{
		const float fx = 1.0f / cameraMatrixColor_.at<double>(0, 0);
		const float fy = 1.0f / cameraMatrixColor_.at<double>(1, 1);
		const float cx = cameraMatrixColor_.at<double>(0, 2);
		const float cy = cameraMatrixColor_.at<double>(1, 2);
		float *it;

		lookupY_ = cv::Mat(1, height, CV_32F);
		it = lookupY_.ptr<float>();
		for(size_t r = 0; r < height; ++r, ++it)
		{
			*it = (r - cy) * fy;
		}

		lookupX_ = cv::Mat(1, width, CV_32F);
		it = lookupX_.ptr<float>();
		for(size_t c = 0; c < width; ++c, ++it)
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
			const float y = lookupY_.at<float>(0, r);
			const float *itX = lookupX_.ptr<float>();
			for(size_t c = 0; c < (size_t)depth.cols; ++c, ++itP, ++itD, ++itC, ++itX)
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
			itP->x = *itX * depthValue;
			itP->y = y * depthValue;
			itP->b = itC->val[0];
			itP->g = itC->val[1];
			itP->r = itC->val[2];
			itP->a = 0;
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

	typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	cloud->height = tmp_rgb_.rows;
	cloud->width = tmp_rgb_.cols;
	cloud->is_dense = false;
	cloud->points.resize(cloud->height * cloud->width);

	if(depthReg_ == 0){

		cameraMatrixColor_ = (cv::Mat_<double>(3,3) <<  
			1.0607072507083330e+03, 0., 9.5635447181548398e+02,
			0., 1.0586083263054650e+03, 5.1897844298824486e+02,
			0., 0., 1.  );

		cameraMatrixDepth_ = (cv::Mat_<double>(3,3) <<
			3.6753450559472907e+02, 0., 2.4449142359870606e+02,
			0.,3.6659938826108794e+02, 2.0783007950245891e+02,
			0., 0., 1. );

		createLookup(tmp_rgb_.cols, tmp_rgb_.rows);
		
		rotation_ = (cv::Mat_<double>(3,3) <<
			9.9983695759005575e-01, -1.6205694274052811e-02,-7.9645282444769407e-03, 
			1.6266694212988934e-02, 9.9983838631845712e-01, 7.6547961099503467e-03,
       		7.8391897822575607e-03, -7.7831045990485416e-03, 9.9993898333166209e-01 );

		translation_ = (cv::Mat_<double>(1,3) <<
			-5.1927840124258939e-02, -4.5307585220976776e-04, 7.0571985343338605e-05 );

		depthReg_ = new DepthRegistrationCPU(cv::Size(tmp_rgb_.cols, tmp_rgb_.rows),
											 cv::Size(tmp_depth_.cols, tmp_depth_.rows),
										     cv::Size(tmp_depth_.cols, tmp_depth_.rows),
											 0.5f, 20.0f);
		depthReg_->init(cameraMatrixColor_, cameraMatrixDepth_, 
						rotation_, translation_,
						cv::Mat::zeros(tmp_depth_.rows, tmp_depth_.cols, CV_32F),
						cv::Mat::zeros(tmp_depth_.rows, tmp_depth_.cols, CV_32F));
	}

	depthReg_->registerDepth(tmp_depth_, registered_ );
	depthReg_->depthToRGBResolution(registered_, scaled_);
	createCloud(scaled_, tmp_rgb_, cloud);

	return cloud;
  }

private:

	libfreenect2::Freenect2 freenect2_;
	libfreenect2::Freenect2Device * dev_ = 0;
	libfreenect2::SyncMultiFrameListener * listener_ = 0;
	libfreenect2::FrameMap frames_;
	DepthRegistration * depthReg_ = 0;
	int mode_;
	cv::Mat scaled_, registered_, translation_, rotation_;
	libfreenect2::Frame *depth_;
	libfreenect2::Frame *rgb_;
	cv::Mat lookupY_, lookupX_, tmp_depth_, tmp_rgb_, cameraMatrixColor_, cameraMatrixDepth_;

};

}