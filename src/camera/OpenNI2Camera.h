//
// Created by Ginger Li on 9/7/2018.
//

#pragma once

#include <OpenNI.h>
#include "opencv2/opencv.hpp"

using namespace std;
using namespace openni;

class OpenNI2Camera
{
public:
  OpenNI2Camera(){};
  ~OpenNI2Camera(){};

  bool initialize();
  bool update();
  void stop();
  void grabColorFrame(cv::Mat &cv_color_frame);
  void grabDepthFrame(cv::Mat &cv_depth_frame);
  void grabDepthFrameForVisualize(cv::Mat &cv_vis_depth);

private:
  Device device_;
  VideoStream color_stream_, depth_stream_;
  VideoFrameRef color_frame_, depth_frame_;
  VideoMode color_mode_, depth_mode_;
  cv::Mat cv_color_frame_, cv_depth_frame_, cv_vis_depth_;
  int max_depth_, min_depth_;
};