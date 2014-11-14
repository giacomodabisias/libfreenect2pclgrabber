#include <libfreenect2/opengl.h>
#include <signal.h>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/threading.h>
#include <libfreenect2/rgb_packet_stream_parser.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool shutdown = false;  //bad

void sigint_handler(int s)
{
  shutdown = true;
}

namespace Kinect2 {

class Kinect2Grabber
{
public:

  Kinect2Grabber() {
  	glfwInit();
		dev_ = freenect2_.openDefaultDevice();
		listener_ = new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Color | libfreenect2::Frame::Ir | libfreenect2::Frame::Depth);
		if(dev_ == 0)
		{
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

  libfreenect2::Freenect2Device::ColorCameraParams 
  GetColorParams(){
    return dev_->getColorCameraParams();
  }

  libfreenect2::Freenect2Device::IrCameraParams 
  GetIrPArams(){
    return dev_->getIrCameraParams();
  }

  void
  FreeFrames(){
    listener_->release(frames_);
  }

private:

  libfreenect2::Freenect2 freenect2_;
  libfreenect2::Freenect2Device * dev_ = 0;
  libfreenect2::SyncMultiFrameListener * listener_ = 0;
	libfreenect2::FrameMap frames_;

};

}