#pragma once

#include "common_include.h"

namespace NaiveSLAM
{

class PinholeCameraModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  PinholeCameraModel();
  PinholeCameraModel(double fx, double fy, double cx, double cy);
  ~PinholeCameraModel();

  Vector3d world2camera(const Vector3d &p_w, const SE3 &T_c_w); // world space -> cmaera space
  Vector3d camera2world(const Vector3d &p_c, const SE3 &T_c_w); // camera space -> world space
  Vector2d camera2pixel(const Vector3d &p_c);                   // camera space -> image space
  Vector3d pixel2camera(const Vector2d &p_p, double depth = 1); // image space -> cmaera space
  Vector3d pixel2world(const Vector2d &p_p, const SE3 &T_c_w, double depth = 1);
                                                                // image space -> world space
  Vector2d world2pixel(const Vector3d &p_w, const SE3 &T_c_w);  // world space -> image space

  const Mat& getCameraMatrix() const;
private:
  double fx_, fy_, cx_, cy_;
  Mat K_; // camera_matrix_
};
} // namespace NaiveSLAM