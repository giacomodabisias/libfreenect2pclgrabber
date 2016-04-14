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
#ifdef WITH_SERIALIZATION
#include "serialization.h"
#endif

struct PlySaver{

  PlySaver(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, bool binary, bool use_camera, K2G & k2g): 
           cloud_(cloud), binary_(binary), use_camera_(use_camera), k2g_(k2g){}

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
  bool binary_;
  bool use_camera_;
  K2G & k2g_;
};

void KeyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void * data)
{
  std::string pressed = event.getKeySym();
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
    if(pressed == "m")
    {
      s->k2g_.mirror();
    }
#ifdef WITH_SERIALIZATION
    if(pressed == "z")
    {
      if(!(s->k2g_.serialize_status())){
        std::cout << "serialization enabled" << std::endl;
        s->k2g_.enableSerialization();
      }
      else{
        std::cout << "serialization disabled" << std::endl;
        s->k2g_.disableSerialization();
      }
    }
#endif
    if(pressed == "x")
    {
        s->k2g_.storeParameters();
        std::cout << "stored calibration parameters" << std::endl;
    }
  }
}

int main(int argc, char * argv[])
{
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively\n";
  std::cout << "Press \'s\' to store a cloud" << std::endl;
  std::cout << "Press \'x\' to store the calibrations." << std::endl;
#ifdef WITH_SERIALIZATION
  std::cout << "Press \'z\' to start/stop serialization." << std::endl;
#endif
  Processor freenectprocessor = OPENGL;
  std::vector<int> ply_file_indices;

  if(argc > 1){
      freenectprocessor = static_cast<Processor>(atoi(argv[1]));
  }
    
  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud;
  K2G k2g(freenectprocessor);
  std::cout << "getting cloud" << std::endl;
  cloud = k2g.getCloud();

  k2g.printParameters();

  cloud->sensor_orientation_.w() = 0.0;
  cloud->sensor_orientation_.x() = 1.0;
  cloud->sensor_orientation_.y() = 0.0;
  cloud->sensor_orientation_.z() = 0.0;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  PlySaver ps(cloud, false, false, k2g);
  viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);

  cv::Mat color, depth;

  while(!viewer->wasStopped()){

    viewer->spinOnce ();
    std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();

    k2g.get(color, depth, cloud);
    // Showing only color since depth is float and needs conversion
    cv::imshow("color", color);
    int c = cv::waitKey(1);
    
    std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
    std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count() * 1000 << std::endl;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->updatePointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");    	

  }

  k2g.shutDown();
  return 0;
}

