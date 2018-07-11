#include "g2o_solvers.h"
#include "pinhole_camera_model.h"

// implementation details from g2o/types/sba/types_siz_dof_expmap.*

namespace NaiveSLAM
{
void EdgeProjectXYZ2UVPoseOnly::computeError()
{
  const VertexSE3Expmap* pose = static_cast<const VertexSE3Expmap *>(_vertices[0]);
  _error = _measurement - camera_model_->camera2pixel(
                              pose->estimate().map(point_));

  // cout << "_error: " << _error << '\n';
}

void EdgeProjectXYZ2UVPoseOnly::linearizeOplus()
{
  VertexSE3Expmap* pose = static_cast<VertexSE3Expmap *>(_vertices[0]);
  SE3Quat T(pose->estimate());

  Vector3d xyz_trans = T.map(point_);
  auto x = xyz_trans[0];
  auto y = xyz_trans[1];
  auto z = xyz_trans[2];
  auto z_2 = z * z;

  Mat K = camera_model_->getCameraMatrix();
  auto fx = K.at<double>(0, 0);
  auto fy = K.at<double>(1, 1);

  _jacobianOplusXi(0, 0) = x * y / z_2 * fx;
  _jacobianOplusXi(0, 1) = -(1 + (x * x / z_2)) * fx;
  _jacobianOplusXi(0, 2) = y / z * fx;
  _jacobianOplusXi(0, 3) = -1. / z * fx;
  _jacobianOplusXi(0, 4) = 0;
  _jacobianOplusXi(0, 5) = x / z_2 * fx;

  _jacobianOplusXi(1, 0) = (1 + y * y / z_2) * fy;
  _jacobianOplusXi(1, 1) = -x * y / z_2 * fy;
  _jacobianOplusXi(1, 2) = -x / z * fy;
  _jacobianOplusXi(1, 3) = 0;
  _jacobianOplusXi(1, 4) = -1. / z * fy;
  _jacobianOplusXi(1, 5) = y / z_2 * fy;

}
} // namespace NaiveSLAM