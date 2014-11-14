#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "Kinect2Grabber.h"
#include "GL/glew.h"



int main(int argc, char *argv[])
{
  
  Kinect2::Kinect2Grabber k2g;
  libfreenect2::RgbPacketStreamParser *rgb_parser = new  libfreenect2::RgbPacketStreamParser();
  char * buffer[1920*1080*3+sizeof(libfreenect2::RgbPacket)];
  //buffer = rgb_parser->buffer_.front();

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
  }
  k2g.ShutDown();
  return 0;
}