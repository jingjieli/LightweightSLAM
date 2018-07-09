#include "OpenNI2Camera.h"

bool OpenNI2Camera::initialize()
{
  // 1. Initial OpenNI
  if (OpenNI::initialize() != STATUS_OK)
  {
    cerr << "OpenNI Initial Error: "
         << OpenNI::getExtendedError() << endl;
    return false;
  }

  // 2. Open Device
  if (device_.open(ANY_DEVICE) != STATUS_OK)
  {
    cerr << "Can't Open Device: "
         << OpenNI::getExtendedError() << endl;
    return false;
  }

  // 3. Create depth stream
  if (device_.hasSensor(SENSOR_DEPTH))
  {
    if (depth_stream_.create(device_, SENSOR_DEPTH) == STATUS_OK)
    {
      // 3a. set video mode
      depth_mode_.setResolution(640, 480);
      depth_mode_.setFps(30);
      depth_mode_.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM);

      if (depth_stream_.setVideoMode(depth_mode_) != STATUS_OK)
      {
        cout << "Can't apply VideoMode: "
             << OpenNI::getExtendedError() << endl;
        return false;
      }
    }
    else
    {
      cerr << "Can't create depth stream on device: "
           << OpenNI::getExtendedError() << endl;
      return false;
    }
  }
  else
  {
    cerr << "ERROR: This device does not have depth sensor" << endl;
    return false;
  }

  // 4. Create color stream
  if (device_.hasSensor(SENSOR_COLOR))
  {
    if (color_stream_.create(device_, SENSOR_COLOR) == STATUS_OK)
    {
      // 4a. set video mode
      color_mode_.setResolution(640, 480);
      color_mode_.setFps(30);
      color_mode_.setPixelFormat(PIXEL_FORMAT_RGB888);

      if (color_stream_.setVideoMode(color_mode_) != STATUS_OK)
      {
        cout << "Can't apply VideoMode: "
             << OpenNI::getExtendedError() << endl;
        return false;
      }

      // 4b. image registration, align depth to color frame
      if (device_.isImageRegistrationModeSupported(
              IMAGE_REGISTRATION_DEPTH_TO_COLOR))
      {
        device_.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR);
      }
    }
    else
    {
      cerr << "Can't create color stream on device: "
           << OpenNI::getExtendedError() << endl;
      return false;
    }
  }

  depth_stream_.start();
  color_stream_.start();

  max_depth_ = depth_stream_.getMaxPixelValue();
  min_depth_ = depth_stream_.getMinPixelValue();

  cv_color_frame_ = cv::Mat(480, 640, CV_8UC3);
  cv_depth_frame_ = cv::Mat(480, 640, CV_16SC1);
  cv_vis_depth_ = cv::Mat(480, 640, CV_8UC1);

  return true;
}

bool OpenNI2Camera::update()
{
  // 5. check is color stream is available
  if (!color_stream_.isValid() || !depth_stream_.isValid())
  {
    cerr << "Invalid stream: "
         << OpenNI::getExtendedError() << endl;
    return false;
  }

  if (color_stream_.readFrame(&color_frame_) != STATUS_OK ||
      depth_stream_.readFrame(&depth_frame_) != STATUS_OK)
  {
    cerr << "Failed to read frame: "
         << OpenNI::getExtendedError() << endl;
    return false;
  }
  // 5a. get color frame
  // 5b. convert data to OpenCV format
  const cv::Mat rgb_image(
      color_frame_.getHeight(), color_frame_.getWidth(),
      CV_8UC3, (void *)color_frame_.getData());
  // 5c. convert form RGB to BGR
  cv::Mat bgr_image;
  cv::cvtColor(rgb_image, bgr_image, CV_RGB2BGR);
  // 5d. flip image
  cv::flip(bgr_image, bgr_image, 1);

  bgr_image.copyTo(cv_color_frame_);

  // 6a. get depth frame
  // 6b. convert data to OpenCV format
  const cv::Mat depth_image(
      depth_frame_.getHeight(), depth_frame_.getWidth(),
      CV_16SC1, (void *)depth_frame_.getData());

  // 6c. flip image
  cv::flip(depth_image, depth_image, 1);
  
  // 6d. re-map depth data [0,Max] to [0,255]
  cv::Mat scaled_depth;
  depth_image.convertTo(scaled_depth, CV_8U, 255.0 / (max_depth_ - min_depth_));

  depth_image.copyTo(cv_depth_frame_);
  scaled_depth.copyTo(cv_vis_depth_);

  return true;
}

void OpenNI2Camera::grabColorFrame(cv::Mat &cv_color_frame)
{
  assert(cv_color_frame.cols == cv_color_frame_.cols &&
         cv_color_frame.rows == cv_color_frame_.rows &&
         cv_color_frame.type() == cv_color_frame_.type());

  cv_color_frame_.copyTo(cv_color_frame);
}

void OpenNI2Camera::grabDepthFrame(cv::Mat &cv_depth_frame)
{
  assert(cv_depth_frame.cols == cv_depth_frame_.cols &&
         cv_depth_frame.rows == cv_depth_frame_.rows &&
         cv_depth_frame.type() == cv_depth_frame_.type());

  cv_depth_frame_.copyTo(cv_depth_frame);
}

void OpenNI2Camera::grabDepthFrameForVisualize(cv::Mat &cv_vis_depth)
{
  assert(cv_vis_depth.cols == cv_vis_depth_.cols &&
         cv_vis_depth.rows == cv_vis_depth_.rows &&
         cv_vis_depth.type() == cv_vis_depth_.type());

  cv_vis_depth_.copyTo(cv_vis_depth);
}

void OpenNI2Camera::stop()
{
  depth_stream_.destroy();
  color_stream_.destroy();
  device_.close();
  OpenNI::shutdown();
}