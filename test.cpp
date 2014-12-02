#include <iostream>
#include "Kinect2Grabber.h"
#include "GL/glew.h"


int minIr, maxIr;

void convertIr(const cv::Mat &ir, cv::Mat &grey, const int min, const int max)
  {
    const float factor = 255.0f / (max - min);
    grey.create(ir.rows, ir.cols, CV_8U);

    #pragma omp parallel for
    for(size_t r = 0; r < (size_t)ir.rows; ++r)
    {
      const uint16_t *itI = ir.ptr<uint16_t>(r);
      uint8_t *itO = grey.ptr<uint8_t>(r);

      for(size_t c = 0; c < (size_t)ir.cols; ++c, ++itI, ++itO)
      {
        *itO = std::min(std::max(*itI - min, 0) * factor, 255.0f);
      }
    }
  }

void findMinMax(const cv::Mat &ir)
  {
    minIr = 0xFFFF;
    maxIr = 0;

    for(size_t r = 0; r < (size_t)ir.rows; ++r)
    {
      const uint16_t *it = ir.ptr<uint16_t>(r);

      for(size_t c = 0; c < (size_t)ir.cols; ++c, ++it)
      {
        minIr = std::min(minIr, (int) * it);
        maxIr = std::max(maxIr, (int) * it);
      }
    }
  }

int main(int argc, char *argv[])
{
  
  //Kinect2::Kinect2Grabber<pcl::PointXYZRGB> k2g("./images/rgb/", "./images/ir/", 16, cv::Size(6,9), 0.025 );
    Kinect2::Kinect2Grabber<pcl::PointXYZRGB> k2g("./rgb_calibration.yaml", "./depth_calibration.yaml", "./pose_calibration.yaml");

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;

  cloud = k2g.GetCloud();

  cloud->sensor_orientation_.w () = 0.0;
  cloud->sensor_orientation_.x () = 1.0;
  cloud->sensor_orientation_.y () = 0.0;
  cloud->sensor_orientation_.z () = 0.0;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  
  while (!viewer->wasStopped ()) {
    viewer->spinOnce ();
    cloud = k2g.GetCloud();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud,rgb, "sample cloud"); 

  }
  /*

  cv::Size boardSize(6,9);
  int count = 0;
  int counter = 0;
  cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(4.0, cv::Size(8, 8));

  
  while(!shutdown)
  {
    
    libfreenect2::FrameMap * frames = k2g.GetRawFrames();
    libfreenect2::Frame *rgb = (*frames)[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = (*frames)[libfreenect2::Frame::Ir];

    cv::Mat ir_gray, ir_scaled, rgb_gray;

    cv::Mat rgb_image = cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data);
    cv::Mat ir_image = cv::Mat(ir->height, ir->width, CV_32FC1, ir->data);

    cv::cvtColor(rgb_image, rgb_gray, 7); // rgb2gray

    cv::resize(ir_image, ir_scaled, cv::Size(), 2.0, 2.0, cv::INTER_CUBIC);
    //convertIr(ir_scaled, ir_gray, 0, 0x7FFF);
    ir_scaled.convertTo(ir_gray, CV_8U, 255.0 / 0x7FFF);
    clahe->apply(ir_gray, ir_gray);

    std::vector<cv::Point2f > camera1ImagePoints;
    bool found1 = cv::findChessboardCorners(rgb_gray, boardSize, camera1ImagePoints, cv::CALIB_CB_FAST_CHECK);
    
    std::vector<cv::Point2f> camera2ImagePoints;
    bool found2 = cv::findChessboardCorners(ir_gray, boardSize, camera2ImagePoints, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
    counter++;
    if(cv::waitKey(30) && (counter >= 50) ){
      counter = 0;
      std::string ir_name = std::string("./images/ir_image_") + std::to_string(count) + std::string(".jpg");
      std::string rgb_name = std::string("./images/rgb_image_") + std::to_string(count) + std::string(".jpg");
      std::cout << "saving image " << ++count << std::endl;
      imwrite( ir_name, ir_gray );
      imwrite( rgb_name, rgb_gray );
      if(count == 16)
        shutdown = true;
    }

    if(found1){
      cv::cornerSubPix(rgb_gray, camera1ImagePoints, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
      drawChessboardCorners(rgb_image, boardSize, camera1ImagePoints, found1);
    }

   if(found2){
      cv::cornerSubPix(ir_gray, camera2ImagePoints, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
      drawChessboardCorners(ir_gray, boardSize, camera2ImagePoints, found2);

    }
    cv::imshow("rgb", rgb_image);
    cv::imshow("ir", ir_gray  );
    
   
    
    std::string rgb_name = std::string("./images/rgb/rgb_image_") + std::to_string(count) + std::string(".jpg");
    std::string ir_name = std::string("./images/ir/ir_image_") + std::to_string(count) + std::string(".jpg");
    cv::Mat rgb_gray = cv::imread(rgb_name, 0);
    cv::Mat ir_gray = cv::imread(ir_name, 0);


    std::vector<cv::Point2f > camera1ImagePoints;
    bool found1 = cv::findChessboardCorners(rgb_gray, boardSize, camera1ImagePoints, cv::CALIB_CB_FAST_CHECK);
    
    std::vector<cv::Point2f> camera2ImagePoints;
    bool found2 = cv::findChessboardCorners(ir_gray, boardSize, camera2ImagePoints, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);
    
    cv::cornerSubPix(rgb_gray, camera1ImagePoints, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
    cv::cornerSubPix(ir_gray, camera2ImagePoints, cv::Size(11, 11), cv::Size(-1, -1), termCriteria);
    
  //  drawChessboardCorners(rgb_gray, boardSize, camera1ImagePoints, found1);
   // drawChessboardCorners(ir_gray, boardSize, camera2ImagePoints, found1);

   

   // cv::imshow("rgb", rgb_gray);
   // cv::imshow("ir", ir_gray  );

    int key = cv::waitKey(30);
    count++;
    if(count == 16)
      break;
    
    
  }


*/



  k2g.ShutDown();
  return 0;
}

