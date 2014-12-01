#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "GL/glew.h"
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
#include <string>


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

	Kinect2Grabber( std::string rgb_image_folder_path, std::string depth_image_folder_path, int image_number, cv::Size board_size, double square_size): 
				    cloud_(new pcl::PointCloud<PointT>())
	{
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
		std::cout<< "starting calibration" << std::endl;
		CalibrateCamera(rgb_image_folder_path, depth_image_folder_path, image_number, board_size, square_size);
		std::cout << "finished calibration" <<std::endl;
		std::cout << "device initialized" << std::endl;
		std::cout << "device serial: " << dev_->getSerialNumber() << std::endl;
		std::cout << "device firmware: " << dev_->getFirmwareVersion() << std::endl;
	}

	Kinect2Grabber(std::string rgb_calibration_file, std::string depth_calibration_file ): cloud_(new pcl::PointCloud<PointT>()) 
	{

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

		std::cout << "device initialized" << std::endl;
		std::cout << "device serial: " << dev_->getSerialNumber() << std::endl;
		std::cout << "device firmware: " << dev_->getFirmwareVersion() << std::endl;
	}


	~Kinect2Grabber(){
		this->ShutDown();
	}

	void 
	calcBoardCornerPositions(cv::Size board_size, float square_size_, std::vector<cv::Point3f>& corners)
	{
	    corners.clear();
	        for( int i = 0; i < board_size.height; ++i )
	            for( int j = 0; j < board_size.width; ++j )
	                corners.push_back(cv::Point3f(float( j*square_size_ ), float( i*square_size_ ), 0));
  
	}

	void 
	saveCameraParams( const std::string& filename,
                        cv::Size image_size, cv::Size board_size,
                        float square_size_, float aspect_ratio, int flags,
                        const cv::Mat& camera_matrix, const cv::Mat& distortion,
                        double total_error ){

		 cv::FileStorage fs( filename, cv::FileStorage::WRITE );

		 time_t t;
		 time( &t );
		 struct tm *t2 = localtime( &t );
		 char buf[1024];
		 strftime( buf, sizeof(buf)-1, "%c", t2 );

		 fs << "calibration_time" << buf;

		 fs << "image_width" << image_size.width;
		 fs << "image_height" << image_size.height;
		 fs << "board_width" << board_size.width;
		 fs << "board_height" << board_size.height;
		 fs << "square_size_" << square_size_;

		 if( flags & cv::CALIB_FIX_ASPECT_RATIO )
		     fs << "aspectRatio" << aspect_ratio;

		 if( flags != 0 )
		 {
		     sprintf( buf, "flags: %s%s%s%s",
		         flags & cv::CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
		         flags & cv::CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
		         flags & cv::CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
		         flags & cv::CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
		 }

		 fs << "flags" << flags;

		 fs << "camera_matrix" << camera_matrix;
		 fs << "distortion_coefficients" << distortion;

		 fs << "avg_reprojection_error" << total_error;
 	}

	void
	CalibrateCamera(std::string rgb_image_folder_path, std::string depth_image_folder_path, int image_number, cv::Size board_size, double square_size){

		const cv::TermCriteria term_criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 50, DBL_EPSILON);
		std::vector<std::vector<cv::Point2f>> rgbImagePoints;
  		std::vector<std::vector<cv::Point2f>> irImagePoints;
  		std::vector<cv::Mat> rvecs, tvecs;


  		if(rgb_image_folder_path.back() != '/')
  			rgb_image_folder_path += std::string("/");

  		if(depth_image_folder_path.back() != '/')
  			depth_image_folder_path += std::string("/");

		int count = 0;
		for(int i = 0; i < image_number; ++i, ++count){

			std::string rgb_name = rgb_image_folder_path + std::string("rgb_image_") + std::to_string(count) + std::string(".jpg");
		    std::string ir_name = depth_image_folder_path + std::string("ir_image_") + std::to_string(count) + std::string(".jpg");

		    cv::Mat rgb_gray = cv::imread(rgb_name, 0);
		    cv::Mat ir_gray = cv::imread(ir_name, 0);

		    std::vector<cv::Point2f > camera1ImagePoints;
		    bool found1 = cv::findChessboardCorners(rgb_gray, board_size, camera1ImagePoints, cv::CALIB_CB_FAST_CHECK);
		    
		    std::vector<cv::Point2f> camera2ImagePoints;
		    bool found2 = cv::findChessboardCorners(ir_gray, board_size, camera2ImagePoints, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
		   
		    if(found1){
		    	cv::cornerSubPix(rgb_gray, camera1ImagePoints, cv::Size(11, 11), cv::Size(-1, -1), term_criteria);
		    	rgbImagePoints.push_back(camera1ImagePoints);
		    }
		    if(found2){
			    cv::cornerSubPix(ir_gray, camera2ImagePoints, cv::Size(11, 11), cv::Size(-1, -1), term_criteria);
			    irImagePoints.push_back(camera2ImagePoints);
			}
		}

		std::vector<std::vector<cv::Point3f> > pointsBoard(1);
		calcBoardCornerPositions(board_size, square_size, pointsBoard[0]);
		pointsBoard.resize(image_number,pointsBoard[0]);

		double error_1 = calibrateCamera(pointsBoard, rgbImagePoints, cv::Size(1920,1080), rgb_camera_matrix_, rgb_distortion_,  rvecs,  tvecs);
		saveCameraParams("rgb_calibration.yaml",cv::Size(1920,1080), board_size, square_size, 0, 0, rgb_camera_matrix_, rgb_distortion_, error_1);

		rgb_fx = rgb_camera_matrix_.at<double>(0,0);
		rgb_fy = rgb_camera_matrix_.at<double>(1,1);
		rgb_cx = rgb_camera_matrix_.at<double>(0,2);
		rgb_cy = rgb_camera_matrix_.at<double>(1,2);

		double error_2 = calibrateCamera(pointsBoard, irImagePoints, cv::Size(1024,848), depth_camera_matrix_, depth_distortion_,  rvecs,  tvecs );
		saveCameraParams("depth_calibration.yaml",cv::Size(1024,848), board_size, square_size, 0, 0, depth_camera_matrix_, depth_distortion_, error_2 );

		ir_fx = depth_camera_matrix_.at<double>(0,0);
		ir_fy = depth_camera_matrix_.at<double>(1,1);
		ir_cx = depth_camera_matrix_.at<double>(0,2);
		ir_cy = depth_camera_matrix_.at<double>(1,2);

		std::cout << std::endl;
		std::cout <<"Single Calibration" <<std::endl;
		std::cout << std::endl;
		std::cout << "rgb :" <<std::endl;
		std::cout << std::endl;
		std::cout << "error:" << error_1 <<std::endl;
		std::cout << std::endl;
		std::cout << "Camera Matrix:" <<std::endl ;
		std::cout << rgb_camera_matrix_ << std::endl;
		std::cout << std::endl;
		std::cout << "Ditortion:" <<std::endl ;
		std::cout << rgb_distortion_ << std::endl;
		std::cout << std::endl;

		std::cout << "Depth :" <<std::endl;
		std::cout << std::endl;
		std::cout << "error:" << error_2 <<std::endl;
		std::cout << std::endl;
		std::cout << "Camera Matrix:" <<std::endl ;
		std::cout << std::endl;
		std::cout << depth_camera_matrix_ << std::endl;
		std::cout << std::endl;
		std::cout << "Ditortion:" <<std::endl ;
		std::cout << std::endl;
		std::cout<< depth_distortion_ << std::endl;
		std::cout << std::endl;


		double rms = cv::stereoCalibrate(pointsBoard, rgbImagePoints, irImagePoints,
		                rgb_camera_matrix_, rgb_distortion_,
		                depth_camera_matrix_, depth_distortion_,
		                cv::Size(1920,1080), rotation_, translation_, essential_, fundamental_,
		                cv::CALIB_FIX_INTRINSIC,
		                term_criteria
		                );

		std::cout << std::endl;
		std::cout <<"Stereo Calibration :" <<std::endl;
		std::cout << std::endl;
		std::cout << "error " << rms << std::endl;
		std::cout << std::endl;
		std::cout << "rotation:" << std::endl; 
		std::cout << std::endl;
		std::cout << rotation_ << std::endl;
		std::cout << std::endl;
		std::cout << "translation" << std::endl;
		std::cout << std::endl;
		std::cout << translation_ << std::endl;
		std::cout << std::endl;
		std::cout << "essential:"  << essential_ << std::endl;
		std::cout << std::endl;
		std::cout << "fundamental" <<  fundamental_ << std::endl;
		std::cout << std::endl;
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

	cv::Mat
	GetCameraMatrixColor(){
		return rgb_camera_matrix_;
	}

	cv::Mat
	GetCameraMatrixDepth(){
		return depth_camera_matrix_;
	}

	cv::Mat
	GetRgbDistortion(){
		return rgb_distortion_;
	}

	cv::Mat
	GetDepthDistortion(){
		return depth_distortion_;
	}

	void
	FreeFrames(){
		listener_->release(frames_);
	}

	
	void createLookup()
	{
		const double fx = 1.0 / rgb_fx;
		const double fy = 1.0 / rgb_fy;
		const double cx = rgb_cx;
		const double cy = rgb_cy;
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
			itP->x = ((c - rgb_cx) / rgb_fx) * depthValue;
			itP->y = ((r - rgb_cy) / rgb_fy) * depthValue;
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
/*
		rgb_camera_matrix_ = (cv::Mat_<double>(3,3) <<  
			color_camera_params_.fx,                0.,               color_camera_params_.cx,
					0.,                  color_camera_params_.fy,     color_camera_params_.cy,
			        0.,                             0.,                         1.             );

		depth_camera_matrix_ = (cv::Mat_<double>(3,3) <<
			ir_camera_params_.fx,                   0.,               ir_camera_params_.cx,
			        0.,                  ir_camera_params_.fy,        ir_camera_params_.cy,
			        0.,                             0.,                         1.             );
				rotation_ = (cv::Mat_<double>(3,3) <<
			9.9983695759005575e-01, -1.6205694274052811e-02,-7.9645282444769407e-03, 
			1.6266694212988934e-02, 9.9983838631845712e-01, 7.6547961099503467e-03,
       		7.8391897822575607e-03, -7.7831045990485416e-03, 9.9993898333166209e-01  );

		rotation_ = (cv::Mat_<double>(3,3) <<
			0.9998475568818841, -0.0007242182755274449, 0.01744530037623355,
			 0.0007490890756651889, 0.9999987124352437, -0.00141915235664019,
            -0.01744425013820719, 0.001432004100563398, 0.9998468120174068 );

		
		translation_ = (cv::Mat_<double>(3,1) <<
			0.1927840124258939e-02, -4.5307585220976776e-04, 7.0571985343338605e-05 );

		translation_ = (cv::Mat_<double>(3,1) <<
			-0.05235922222877874, -0.0002122509177667504, -0.00358717077329887 );

		

		distortion_ = (cv::Mat_<double>(1,5) <<
			0.08886683884842322, -0.2474225084881365, -0.002864812899835538, -0.0006690936537519126, 0.05729994721039293);  			
		*/
		depthReg_ = new DepthRegistrationCPU();

		size_registered_ = cv::Size(1920, 1080); 
		size_depth_ = cv::Size(512, 424);

		cloud_->height = size_registered_.height;
		cloud_->width = size_registered_.width;
		cloud_->is_dense = false;
		cloud_->points.resize(cloud_->height * cloud_->width);

		depthReg_->init(rgb_camera_matrix_,
						size_registered_,
					    depth_camera_matrix_,
					    size_depth_,
					    depth_distortion_,
						rotation_, 
						translation_ );
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
	cv::Mat scaled_, registered_;
	cv::Mat rotation_, translation_, essential_, fundamental_;
	libfreenect2::Frame *depth_;
	libfreenect2::Frame *rgb_;
	cv::Size size_registered_, size_depth_;
	cv::Mat lookupY_, lookupX_, tmp_depth_, tmp_rgb_;
	typename pcl::PointCloud<PointT>::Ptr cloud_;
  	cv::Mat rgb_camera_matrix_, depth_distortion_, depth_camera_matrix_, rgb_distortion_;
  	double rgb_fx, rgb_fy, rgb_cx, rgb_cy;
  	double ir_fx, ir_fy, ir_cx, ir_cy;
};

}