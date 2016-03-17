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


struct PlySaver{

  PlySaver(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_1, boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_2, bool binary, bool use_camera, K2G & k2g_1, K2G & k2g_2): 
           cloud_1_(cloud_1), cloud_2_(cloud_2), binary_(binary), use_camera_(use_camera), k2g_1_(k2g_1), k2g_2_(k2g_2){}

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_1_;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_2_;
  K2G & k2g_1_;
  K2G & k2g_2_;
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
      writer.write ("cloud_1_" + now + ".ply", *(s->cloud_1_), s->binary_, s->use_camera_);
      writer.write ("cloud_2_" + now + ".ply", *(s->cloud_2_), s->binary_, s->use_camera_);
      cv::imwrite("color_1_" + now + ".jpg", s->k2g_1_.getColor());
      cv::imwrite("color_2_" + now + ".jpg", s->k2g_2_.getColor());
      
      std::cout << "saved " << "cloud and color " + now << std::endl;
    }
  }
}

int main(int argc, char *argv[])
{
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively" << std::endl;
  std::cout << "followed by the two kinect serials" << std::endl;

  processor freenectprocessor = OPENGL;
  std::vector<int> ply_file_indices;
  if(argc < 4){
    std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively" << std::endl;
    std::cout << "followed by the two kinect serials" << std::endl;
    return -1;
  }

  freenectprocessor = static_cast<processor>(atoi(argv[1]));
  
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_1;
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_2;
  K2G k2g_1(freenectprocessor, argc == 5 ? true : false, argv[2]); //"005157353647"
  K2G k2g_2(freenectprocessor, argc == 5 ? true : false, argv[3]); //"500320441942"

  std::cout << "getting cloud" << std::endl;
  cloud_1 = k2g_1.getCloud();
  cloud_2 = k2g_2.getCloud();

  cloud_1->sensor_orientation_.w() = 0.0;
  cloud_1->sensor_orientation_.x() = 1.0;
  cloud_1->sensor_orientation_.y() = 0.0;
  cloud_1->sensor_orientation_.z() = 0.0;

  cloud_2->sensor_orientation_.w() = 0.0;
  cloud_2->sensor_orientation_.x() = 1.0;
  cloud_2->sensor_orientation_.y() = 0.0;
  cloud_2->sensor_orientation_.z() = 0.0;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_1(new pcl::visualization::PCLVisualizer ("3D Viewer_1"));
  viewer_1->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_1(cloud_1);
  viewer_1->addPointCloud<pcl::PointXYZRGB>(cloud_1, rgb_1, "sample cloud_1");
  viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud_1");

  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_2(cloud_2);
  viewer_1->addPointCloud<pcl::PointXYZRGB>(cloud_2, rgb_2, "sample cloud_2");
  viewer_1->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud_2");

  PlySaver ps_1(cloud_1, cloud_2, false, false, k2g_1, k2g_2);
  viewer_1->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps_1);

  while(!viewer_1->wasStopped()){

    viewer_1->spinOnce ();

    std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();

    cloud_1 = k2g_1.updateCloud(cloud_1);
    cloud_2 = k2g_2.updateCloud(cloud_2);

    std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
    std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count() * 1000 << std::endl;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_1(cloud_1);
    viewer_1->updatePointCloud<pcl::PointXYZRGB> (cloud_1, rgb_1, "sample cloud_1");
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb_2(cloud_2);
    viewer_1->updatePointCloud<pcl::PointXYZRGB> (cloud_2, rgb_2, "sample cloud_2");    	

  }

  k2g_1.shutDown();
  k2g_2.shutDown();
  return 0;
}

