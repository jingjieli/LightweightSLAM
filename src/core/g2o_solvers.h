#pragma once

#include "common_include.h"

namespace NaiveSLAM
{
class PinholeCameraModel;

class EdgeProjectXYZ2UVPoseOnly : public BaseUnaryEdge<2, Vector2d, VertexSE3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void computeError();
  virtual void linearizeOplus();

  bool read(std::istream &is) { return false; }
  bool write(std::ostream &os) const { return false; }

  Vector3d point_;
  PinholeCameraModel *camera_model_;
};

class EdgeProjectXYZRGBDPoseOnly : public BaseUnaryEdge<3, Vector3d, VertexSE3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void computeError();
  virtual void linearizeOplus();

  bool read(std::istream &is) { return false; }
  bool write(std::ostream &os) const { return false; }

  Vector3d point_;
};

class EdgeProjectXYZRGBD : public BaseBinaryEdge<3, Vector3d, VertexSBAPointXYZ, VertexSE3Expmap>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void computeError();
  virtual void linearizeOplus();

  bool read(std::istream &is) { return false; }
  bool write(std::ostream &os) const { return false; }
};

} // namespace NaiveSLAM