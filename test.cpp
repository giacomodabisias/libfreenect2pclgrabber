#include "Kinect2Grabber.hpp"

int main(int argc, char *argv[])
{
  
  //Kinect2::Kinect2Grabber<pcl::PointXYZRGB> k2g("./images/rgb/", "./images/ir/", 16, cv::Size(6,9), 0.025 );
  pcl::Kinect2Grabber::Kinect2Grabber<pcl::PointXYZRGB> k2g("./rgb_calibration.yaml", "./depth_calibration.yaml", "./pose_calibration.yaml");

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

  k2g.ShutDown();
  return 0;
}

