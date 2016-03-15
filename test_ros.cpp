/*
Copyright 2015, Giacomo Dabisias"
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
@Author 
Giacomo  Dabisias, PhD Student
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/
#include "k2g.h"
#include <pcl/visualization/cloud_viewer.h>
#include <chrono>
// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


struct PlySaver{

  PlySaver(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, bool binary, bool use_camera): 
           cloud_(cloud), binary_(binary), use_camera_(use_camera){}

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
  bool binary_;
  bool use_camera_;

};


void
KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data)
{
  std::string pressed;
  pressed = event.getKeySym();
  PlySaver * s = (PlySaver*)data;
  if(event.keyDown ())
  {
    if(pressed == "s")
    {
      
      pcl::PLYWriter writer;
      std::chrono::high_resolution_clock::time_point p = std::chrono::high_resolution_clock::now();
      std::string now = std::to_string((long)std::chrono::duration_cast<std::chrono::milliseconds>(p.time_since_epoch()).count());
      writer.write ("cloud_" + now, *(s->cloud_), s->binary_, s->use_camera_);
      std::cout << "saved " << "cloud_" + now + ".ply" << std::endl;
    }
  }
}



int main(int argc, char *argv[])
{
  
  ros::init (argc, argv, "kinect2grab");
  ros::NodeHandle nh;
  
  ros::Publisher PointCloudPub = nh.advertise<sensor_msgs::PointCloud2>("/k2_points", 1);
  ros::Publisher ColorImagePub = nh.advertise<sensor_msgs::Image>("k2_color", 1);
  
  sensor_msgs::PointCloud2 point_cloud;
  sensor_msgs::Image color_image;
  
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2 correspond to CPU, OPENCL, and OPENGL respectively\n";
  processor freenectprocessor = OPENGL;
  std::vector<int> ply_file_indices;
  
 
  if(argc > 1){
      freenectprocessor = static_cast<processor>(atoi(argv[1]));
  }
    
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  cv::Mat color(1080,1920, CV_8UC4);
  
  K2G k2g(freenectprocessor);
  std::cout << "getting cloud" << std::endl;
  cloud = k2g.getCloud();
  color = k2g.getColor();

  cloud->sensor_orientation_.w() = 0.0;
  cloud->sensor_orientation_.x() = 1.0;
  cloud->sensor_orientation_.y() = 0.0;
  cloud->sensor_orientation_.z() = 0.0;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  PlySaver ps(cloud, false, false);
  viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);
  
  while(!viewer->wasStopped()){

    viewer->spinOnce ();
    std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();
    cloud = k2g.updateCloud(cloud);
    color = k2g.getColor();
    std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
    
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");    
    
    pcl::toROSMsg(*cloud, point_cloud);
    point_cloud.header.frame_id="/base_link";
    point_cloud.header.stamp = ros::Time::now();
    
    sensor_msgs::Image::Ptr color_image;
    std_msgs::Header head;
   
    color_image = cv_bridge::CvImage(head, "bgra8", color).toImageMsg();

    ColorImagePub.publish(color_image);
    PointCloudPub.publish(point_cloud);
  }

  k2g.shutDown();
  return 0;
}