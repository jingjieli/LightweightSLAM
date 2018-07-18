#pragma once

#include "common_include.h"

namespace NaiveSLAM
{

class PinholeCameraModel;

class ReprojectionError
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ReprojectionError(const Vector3d &pt_3d, const Vector2d &pt_2d, PinholeCameraModel *camera_model)
      : pt_3d_(pt_3d), pt_2d_(pt_2d), camera_model_(camera_model)
  {
    K_ = camera_model_->getCameraMatrix();
  }
  ~ReprojectionError(){};

  template <typename T>
  bool operator()(const T * const pose, T *residuals) const
  {
    auto quat = Quaternion<T>(pose[3], pose[0], pose[1], pose[2]);
    auto t = Matrix<T, 3, 1>(pose[4], pose[5], pose[6]);

    Matrix<T, 3, 1> pt_3d_w; // point in world space
    pt_3d_w << T(pt_3d_.x()), T(pt_3d_.y()), T(pt_3d_.z()); 

    auto pt_3d_c = quat * pt_3d_w + t; // point in camera space

    T cam_intrinsic[4] = {
        T(K_.at<double>(0, 0)),
        T(K_.at<double>(1, 1)),
        T(K_.at<double>(0, 2)),
        T(K_.at<double>(1, 2))};

    // camera space -> image space
    T predicted_x = cam_intrinsic[0] * pt_3d_c(0, 0) / pt_3d_c(2, 0) + cam_intrinsic[2];
    T predicted_y = cam_intrinsic[1] * pt_3d_c(1, 0) / pt_3d_c(2, 0) + cam_intrinsic[3];

    residuals[0] = predicted_x - T(pt_2d_.x());
    residuals[1] = predicted_y - T(pt_2d_.y());

    return true;
  }

private:
  Vector3d pt_3d_;
  Vector2d pt_2d_;
  PinholeCameraModel *camera_model_;
  Mat K_;
};

} // namespace NaiveSLAM