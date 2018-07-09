//
// Created by Ginger Li on 8/3/2018.
//

#include "astra_rgbd_camera.h"

AstraRgbdCamera::AstraRgbdCamera()
{
}

AstraRgbdCamera::~AstraRgbdCamera()
{
}

void AstraRgbdCamera::initialize()
{
  astra::initialize(); 
  printf("Call astra::initialize succeed\n");

  stream_reader_ = stream_set_.create_reader();
  stream_reader_.stream<astra::PointStream>().start();

  astra::ColorStream color_stream_ = stream_reader_.stream<astra::ColorStream>();
  astra::DepthStream depth_stream_ = stream_reader_.stream<astra::DepthStream>();

  configureDepth(depth_stream_);
  configureColor(color_stream_);

  depth_stream_.start();
  color_stream_.start();

  stream_reader_.add_listener(frame_listener_);
}

void AstraRgbdCamera::update()
{
  astra_update();
}

void AstraRgbdCamera::stop()
{
  astra::terminate();
}

void AstraRgbdCamera::grabColorFrame(cv::Mat &cv_color_frame)
{
  frame_listener_.getColorFrame(cv_color_frame);
}

void AstraRgbdCamera::grabDepthFrame(cv::Mat &cv_depth_frame)
{
  frame_listener_.getDepthFrame(cv_depth_frame);
}

void AstraRgbdCamera::grabPointFrame(cv::Mat &cv_point_frame)
{
  frame_listener_.getPointFrame(cv_point_frame);
}

void AstraRgbdCamera::configureColor(astra::ColorStream &color_stream)
{
  astra::ImageStreamMode color_mode;
  color_mode.set_width(640);
  color_mode.set_height(480);
  color_mode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_RGB888);
  color_mode.set_fps(30);

  color_stream.set_mode(color_mode);
}

void AstraRgbdCamera::configureDepth(astra::DepthStream &depth_stream)
{
  //We don't have to set the mode to start the stream, but if you want to here is how:
  astra::ImageStreamMode depth_mode;

  depth_mode.set_width(640);
  depth_mode.set_height(480);
  depth_mode.set_pixel_format(astra_pixel_formats::ASTRA_PIXEL_FORMAT_DEPTH_MM);
  depth_mode.set_fps(30);

  depth_stream.set_mode(depth_mode);
}