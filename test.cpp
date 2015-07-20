//Copyright 2014 Giacomo Dabisias
//
//Licensed under the Apache License, Version 2.0 (the "License");
//you may not use this file except in compliance with the License.
//You may obtain a copy of the License at
//
//   http://www.apache.org/licenses/LICENSE-2.0
//
//Unless required by applicable law or agreed to in writing, software
//distributed under the License is distributed on an "AS IS" BASIS,
//WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//See the License for the specific language governing permissions and
//limitations under the License.
//This is preliminary software and/or hardware and APIs are preliminary and subject to change.
#include "Kinect2Grabber.hpp"
#include <chrono>

int main(int argc, char *argv[])
{
  //The first constructor calibrates the cameras using images inside the specified folders.
  //The parameters are: rgb image folder, depth image folder, number of images, size of the checkboard, size of the check board squares in m
  //The second constructor load an existing calibration
  
  //Kinect2Grabber::Kinect2Grabber<pcl::PointXYZRGB> k2g("./images512/rgb/", "./images512/ir/", 16, cv::Size(6,9), 0.025 );
  Kinect2Grabber::Kinect2Grabber<pcl::PointXYZRGB> k2g("../calibration/rgb_calibration.yaml", "../calibration/depth_calibration.yaml", "../calibration/pose_calibration.yaml");
  //Kinect2Grabber::Kinect2Grabber<pcl::PointXYZRGB> k2g;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  k2g.setDistance(10000);
  std::cout << "getting cloud" << std::endl;
  cloud = k2g.getFullCloud();
  //cloud = k2g.getCloud();

  cloud->sensor_orientation_.w() = 0.0;
  cloud->sensor_orientation_.x() = 1.0;
  cloud->sensor_orientation_.y() = 0.0;
  cloud->sensor_orientation_.z() = 0.0;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  while (!viewer->wasStopped ()) {
    viewer->spinOnce ();
    using namespace std::chrono;
    static high_resolution_clock::time_point last;

    auto tnow = high_resolution_clock::now();
    cloud = k2g.getFullCloud();

    //cloud = k2g.getCloud();
    auto tpost = high_resolution_clock::now();
    std::cout << "delta " << duration_cast<duration<double>>(tpost-last).count()*1000 << " " << duration_cast<duration<double>>(tpost-tnow).count()*1000 << std::endl;
    last = tpost;
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud"); 
  }

  k2g.shutDown();
  return 0;
}

