#include "pinhole_camera_model.h"

namespace NaiveSLAM
{
PinholeCameraModel::PinholeCameraModel()
{
}

PinholeCameraModel::PinholeCameraModel(double fx, double fy, double cx, double cy)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy)
{
  K_ = (Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
}

PinholeCameraModel::~PinholeCameraModel()
{
}

Vector3d PinholeCameraModel::world2camera(const Vector3d &p_w, const SE3 &T_c_w)
{
  return T_c_w * p_w;
}
Vector3d PinholeCameraModel::camera2world(const Vector3d &p_c, const SE3 &T_c_w)
{
  return T_c_w.inverse() * p_c;
}
Vector2d PinholeCameraModel::camera2pixel(const Vector3d &p_c)
{
  return Vector2d{
      fx_ * p_c(0, 0) / p_c(2, 0) + cx_,
      fy_ * p_c(1, 0) / p_c(2, 0) + cy_};
}
Vector3d PinholeCameraModel::pixel2camera(const Vector2d &p_p, double depth)
{
  return Vector3d{
      (p_p(0, 0) - cx_) * depth / fx_,
      (p_p(1, 0) - cy_) * depth / fy_,
      depth};
}
Vector3d PinholeCameraModel::pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth)
{
  return camera2world(pixel2camera(p_p, depth), T_c_w);
}
Vector2d PinholeCameraModel::world2pixel(const Vector3d &p_w, const SE3 &T_c_w)
{
  return camera2pixel(world2camera(p_w, T_c_w));
}

const Mat& PinholeCameraModel::getCameraMatrix() const
{
  return K_;
}
} // namespace NaiveSLAM
