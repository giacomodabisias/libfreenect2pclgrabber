/*
Copyright 2016, Giacomo Dabisias & Michele Mambrini"
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
Giacomo  Dabisias, PhD Student & Michele Mambrini
PERCRO, (Laboratory of Perceptual Robotics)
Scuola Superiore Sant'Anna
via Luigi Alamanni 13D, San Giuliano Terme 56010 (PI), Italy
*/
#include "k2g_ros.h"

// extra headers for writing out ply file
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/io/ply_io.h>

struct PlySaver{

	PlySaver(boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud, bool binary, bool use_camera, K2GRos & ros_grabber): 
           cloud_(cloud), binary_(binary), use_camera_(use_camera), K2G_ros_(ros_grabber){}

 	boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_;
	bool binary_;
  	bool use_camera_;
  	K2GRos & K2G_ros_;
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
			s->K2G_ros_.mirror();
		}
		if(pressed == "e")
		{
			s->K2G_ros_.setShutdown();
			std::cout << "SHUTTING DOWN" << std::endl;
		}
	}
}


int main(int argc, char * argv[])
{
	std::cout << "Syntax is: " << argv[0] << " [-processor 0|1|2] -processor options 0,1,2,3 correspond to CPU, OPENCL, OPENGL, CUDA respectively\n";
	Processor freenectprocessor = OPENGL;

	if(argc > 1){
		freenectprocessor = static_cast<Processor>(atoi(argv[1]));
	}

	ros::init(argc, argv, "RosKinect2Grabber");

	K2GRos K2G_ros(freenectprocessor);

	while((ros::ok()) && (!K2G_ros.terminate()))
	{  		
		K2G_ros.publishColor();
		K2G_ros.publishCameraInfoColor();
		K2G_ros.publishCameraInfoDepth(); 
		
	}

	K2G_ros.shutDown();
	return 0;
}

