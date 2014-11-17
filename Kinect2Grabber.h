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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

 

  Kinect2Grabber(int mode): mode_(mode), init_(true) {
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
    typename pcl::PointCloud<PointT>::Ptr cloud;
    if(init_){
      cameraMatrixColor_ = (cv::Mat_<double>() <<  1.0660120360637927e+03, 0., 9.4558752751085558e+02, 0.,
       1.0688708399911650e+03, 5.2006994012356529e+02, 0., 0., 1. );
      cameraMatrixDepth_ = (cv::Mat_<double>() <<  3.6638346288148574e+02, 0., 2.5564531890330468e+02, 0.,
       3.6714380707081017e+02, 2.0398020160452000e+02, 0., 0., 1. );
      createLookup(tmp_rgb_.cols, tmp_rgb_.rows);
      //if(mode_ == 0)
      //  depthReg_ = DepthRegistration::New(cv::Size(rgb_.cols, rgb_.rows), cv::Size(depth_.cols, depth_.rows), cv::Size(depth_.cols, depth_.rows), 0.5f, 20.0f, 0.015f, DepthRegistration::OPENCL);
      //else
        depthReg_ = DepthRegistration::New(cv::Size(tmp_rgb_.cols, tmp_rgb_.rows),
                    cv::Size(tmp_depth_.cols, tmp_depth_.rows),
                    cv::Size(tmp_depth_.cols, tmp_depth_.rows),
                    0.5f, 20.0f, 0.015f, DepthRegistration::CPU);
      
      depthReg_->init(cameraMatrixColor_, cameraMatrixDepth_, 
                    cv::Mat::eye(3, 3, CV_64F), cv::Mat::zeros(1, 3, CV_64F),
                    cv::Mat::zeros(tmp_depth_.rows, tmp_depth_.cols, CV_32F),
                    cv::Mat::zeros(tmp_depth_.rows, tmp_depth_.cols, CV_32F));
    }
    
    depthReg_->depthToRGBResolution(tmp_depth_, scaled_);
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
  bool init_;
  cv::Mat scaled_;
  libfreenect2::Frame *depth_;
  libfreenect2::Frame *rgb_;
  cv::Mat  lookupY_, lookupX_, tmp_depth_, tmp_rgb_, cameraMatrixColor_, cameraMatrixDepth_;

};

}