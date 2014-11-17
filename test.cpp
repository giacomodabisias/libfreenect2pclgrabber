#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "Kinect2Grabber.h"
#include "GL/glew.h"




int main(int argc, char *argv[])
{
  
  Kinect2::Kinect2Grabber<pcl::PointXYZRGB> k2g(1);
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
  /*
  while(!shutdown)
  {
    
    libfreenect2::FrameMap * frames = k2g.GetRawFrames();
    libfreenect2::Frame *rgb = (*frames)[libfreenect2::Frame::Color];
    libfreenect2::Frame *ir = (*frames)[libfreenect2::Frame::Ir];
    libfreenect2::Frame *depth = (*frames)[libfreenect2::Frame::Depth];
    cv::imshow("rgb", cv::Mat(rgb->height, rgb->width, CV_8UC3, rgb->data));
    cv::imshow("ir", cv::Mat(ir->height, ir->width, CV_32FC1, ir->data) / 20000.0f);
    cv::imshow("depth", cv::Mat(depth->height, depth->width, CV_32FC1, depth->data) / 4500.0f);
    k2g.FreeFrames();
    int key = cv::waitKey(1);
    shutdown = shutdown || (key > 0 && ((key & 0xFF) == 27)); // shutdown on escape
    
  }*/

  while (!viewer->wasStopped ()) {
    viewer->spinOnce ();
    cloud = k2g.GetCloud();
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud,rgb, "sample cloud"); 

  }

  k2g.ShutDown();
  return 0;
}