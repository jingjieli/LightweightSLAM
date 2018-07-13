#pragma once

#include "common_include.h"

namespace NaiveSLAM
{

class MapPoint;
class Map;

class Frame
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Frame();
  Frame(const SE3 &T_c_w);
  ~Frame();
  
  static unique_ptr<Frame> createFrame(const SE3 &T_c_w);

  unsigned long getFrameId() const;
  SE3 getTransform() const;
  void setTransform(const SE3 &new_T_c_w);
  Vector3d getCamCenter() const;

private:
  static unsigned long factory_id_;
  unsigned long frame_id_;
  SE3 T_c_w_; // transform from world to camera
  vector<KeyPoint*> keypoints_;
  vector<MapPoint*> map_points_;
};

} // namespace NaiveSLAM