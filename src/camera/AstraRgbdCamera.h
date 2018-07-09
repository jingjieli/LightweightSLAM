//
// Created by Ginger Li on 8/3/2018.
//

#ifndef ASTRA_RGBD_CAMERA_H
#define ASTRA_RGBD_CAMERA_H

#include <mutex>
#include "astra/astra.hpp"
#include "opencv2/opencv.hpp"

class MultiFrameListener : public astra::FrameListener
{

public:
  void updatePoint(astra::Frame &frame)
  {
    const astra::PointFrame pointFrame = frame.get<astra::PointFrame>();

    if (!pointFrame.is_valid())
      return;

    const int pointWidth = pointFrame.width();
    const int pointHeight = pointFrame.height();

    const astra::Vector3f *points = pointFrame.data();
    float point_buffer[pointWidth * pointHeight * 3];

    for (int i = 0; i < pointWidth * pointHeight; i++)
    {
      const int xyzOffset = i * 3;
      point_buffer[xyzOffset] = points[i].x;
      point_buffer[xyzOffset + 1] = points[i].y;
      point_buffer[xyzOffset + 2] = points[i].z;
    }

    cv::Mat cv_point_frame(pointHeight, pointWidth, CV_32FC3, &point_buffer);

    data_mtx.lock();
    curr_point_frame = cv_point_frame.clone();
    data_mtx.unlock();
  }

  void updateDepth(astra::Frame &frame)
  {
    const astra::DepthFrame depthFrame = frame.get<astra::DepthFrame>();

    if (!depthFrame.is_valid())
      return;

    const int depthWidth = depthFrame.width();
    const int depthHeight = depthFrame.height();

    const int16_t *depths = depthFrame.data();
    int16_t depth_buffer[depthWidth * depthHeight];

    for (int i = 0; i < depthWidth * depthHeight; i++)
    {
      depth_buffer[i] = depths[i];
    }

    cv::Mat cv_depth_frame(depthHeight, depthWidth, CV_16SC1, &depth_buffer);

    data_mtx.lock();
    curr_depth_frame = cv_depth_frame.clone();
    data_mtx.unlock();
  }

  void updateColor(astra::Frame &frame)
  {
    const astra::ColorFrame colorFrame = frame.get<astra::ColorFrame>();

    if (!colorFrame.is_valid())
      return;

    const int colorWidth = colorFrame.width();
    const int colorHeight = colorFrame.height();

    const astra::RgbPixel *color = colorFrame.data();
    //uint8_t* buffer = &colorView_.buffer_[0];

    //uint8_t buffer[colorWidth * colorHeight * 4];
    unsigned char uchar_buffer[colorWidth * colorHeight * 4];

    for (int i = 0; i < colorWidth * colorHeight; i++)
    {
      // const int rgbaOffset = i * 4;
      // buffer[rgbaOffset] = color[i].r;
      // buffer[rgbaOffset + 1] = color[i].g;
      // buffer[rgbaOffset + 2] = color[i].b;
      // buffer[rgbaOffset + 3] = 255;

      const int rgbaOffset = i * 4;
      uchar_buffer[rgbaOffset] = color[i].b;
      uchar_buffer[rgbaOffset + 1] = color[i].g;
      uchar_buffer[rgbaOffset + 2] = color[i].r;
      uchar_buffer[rgbaOffset + 3] = 255;
    }

    cv::Mat cv_color_frame(colorHeight, colorWidth, CV_8UC4, &uchar_buffer);

    data_mtx.lock();
    curr_color_frame = cv_color_frame.clone();
    data_mtx.unlock();
  }

  void checkFps()
  {
    // const float frameWeight = .2f;

    // const std::chrono::high_resolution_clock::time_point now = ClockType::now();
    // const float elapsedMillis = std::chrono::duration_cast<std::chrono::milliseconds>(now - prev_).count();

    // elapsedMillis_ = elapsedMillis * frameWeight + elapsedMillis_ * (1.f - frameWeight);
    // prev_ = now;

    // const float fps = 1000.f / elapsedMillis;

    // const auto precision = std::cout.precision();

    // std::cout << std::fixed
    //           << std::setprecision(1)
    //           << fps << " fps ("
    //           << std::setprecision(1)
    //           << elapsedMillis_ << " ms)"
    //           << std::setprecision(precision)
    //           << std::endl;
  }

  void getColorFrame(cv::Mat &color_frame)
  {
    data_mtx.lock();
    color_frame = curr_color_frame.clone();
    data_mtx.unlock();
  }

  void getPointFrame(cv::Mat &point_frame)
  {
    data_mtx.lock();
    point_frame = curr_point_frame.clone();
    data_mtx.unlock();
  }

  void getDepthFrame(cv::Mat &depth_frame)
  {
    data_mtx.lock();
    depth_frame = curr_depth_frame.clone();
    data_mtx.unlock();
  }

  virtual void on_frame_ready(astra::StreamReader &reader,
                              astra::Frame &frame) override
  {
    updatePoint(frame);
    updateDepth(frame);
    updateColor(frame);
    checkFps();
  }

private:
  cv::Mat curr_color_frame;
  cv::Mat curr_depth_frame;
  cv::Mat curr_point_frame;
  std::mutex data_mtx;
};

class AstraRgbdCamera
{
public:
  AstraRgbdCamera();
  ~AstraRgbdCamera();

  void initialize();
  void update();
  void stop();
  void grabColorFrame(cv::Mat &cv_color_frame);
  void grabDepthFrame(cv::Mat &cv_depth_frame);
  void grabPointFrame(cv::Mat &cv_point_frame);

private:
  astra::StreamSet stream_set_;
  astra::StreamReader stream_reader_;
  MultiFrameListener frame_listener_;

  void configureColor(astra::ColorStream &color_stream);
  void configureDepth(astra::DepthStream &depth_stream);
};

#endif // ASTRA_RGBD_CAMERA_H