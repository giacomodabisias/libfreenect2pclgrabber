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

  PlySaver(std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> & clouds, bool binary, bool use_camera, std::vector<K2G *> & kinects): 
           binary_(binary), use_camera_(use_camera), clouds_(clouds), kinects_(kinects){}

  std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> & clouds_;
  std::vector<K2G *> & kinects_;
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
      for(size_t i = 0; i < s->kinects_.size(); ++i){
      	writer.write ("cloud_"+ std::to_string(i) + "_" + now + ".ply", *(s->clouds_[i]), s->binary_, s->use_camera_);
      	cv::imwrite("color_" + std::to_string(i) + "_" + now + ".jpg", s->kinects_[i]->getColor());
      }
      std::cout << "saved " << "cloud and color " + now << std::endl;
    }
    if(pressed == "x")
    {
        for(auto & k : s->kinects_)
        	k->storeParameters();
        std::cout << "stored calibration parameters" << std::endl;
    }
  }
}

int main(int argc, char *argv[])
{
  std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively" << std::endl;
  std::cout << "followed by the kinect2 serials and a last 1 for mirroring" << std::endl;
  std::cout << "Press \'s\' to store both clouds and color images." << std::endl;
  std::cout << "Press \'x\' to store both calibrations." << std::endl;

  processor freenectprocessor = OPENGL;
  std::vector<int> ply_file_indices;
  if(argc < 3){
    std::cout << "Wrong syntax! specify at least processor and one serial" << std::endl;
    return -1;
  }

  freenectprocessor = static_cast<processor>(atoi(argv[1]));
  bool mirroring = atoi(argv[argc-1]) == 1 ? true : false;

  if(mirroring)
  	std::cout << "mirroring enabled" << std::endl;

  int kinect2_count = mirroring ? argc - 3 : argc - 2;
  std::cout << "loading " << kinect2_count << " devices" << std::endl;

  std::vector<K2G *> kinects(kinect2_count);
  std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>>> clouds(kinect2_count);

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D viewer"));
  viewer->setBackgroundColor (0, 0, 0);

  for(size_t i = 0; i < kinect2_count; ++i)
  {
  	kinects[i] = new K2G(freenectprocessor, mirroring, argv[i + 2]);
  	clouds[i] = kinects[i]->getCloud();

  	clouds[i]->sensor_orientation_.w() = 0.0;
  	clouds[i]->sensor_orientation_.x() = 1.0;
  	clouds[i]->sensor_orientation_.y() = 0.0;
  	clouds[i]->sensor_orientation_.z() = 0.0; 

  	viewer->addPointCloud<pcl::PointXYZRGB>(clouds[i], pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB>(clouds[i]), "sample cloud_" + i);
  	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud_" + i);
  }

  PlySaver ps(clouds, false, false, kinects);
  viewer->registerKeyboardCallback(KeyboardEventOccurred, (void*)&ps);
  std::cout << "starting cycle" << std::endl;
  while(!viewer->wasStopped()){

    viewer->spinOnce ();

    std::chrono::high_resolution_clock::time_point tnow = std::chrono::high_resolution_clock::now();

    for(size_t i = 0; i < kinect2_count; ++i)
    	clouds[i] = kinects[i]->updateCloud(clouds[i]);

    std::chrono::high_resolution_clock::time_point tpost = std::chrono::high_resolution_clock::now();
    std::cout << "delta " << std::chrono::duration_cast<std::chrono::duration<double>>(tpost-tnow).count() * 1000 << std::endl;

    for(size_t i = 0; i < kinect2_count; ++i)
    	viewer->updatePointCloud<pcl::PointXYZRGB> (clouds[i], pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> (clouds[i]), "sample cloud_" + i);
  }

  for(auto & k : kinects)
  	k->shutDown();

  return 0;
}

