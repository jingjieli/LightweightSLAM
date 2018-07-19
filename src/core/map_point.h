#pragma once

#include "common_include.h"

namespace NaiveSLAM
{

class Frame;
class Map;

class MapPoint
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MapPoint();
  MapPoint(const Vector3d &world_coord, const Mat &descriptor, shared_ptr<Frame> frame);
  ~MapPoint();

  static unique_ptr<MapPoint> createMapPoint(const Vector3d &world_coord, const Mat &descriptor, shared_ptr<Frame> frame);
  
  unsigned long getMapPointId() const;
  const Vector3d& getPosition() const;
  const Mat& getDescriptor() const;

  unsigned int visible_times_;
  unsigned int matched_times_;

private:
  static unsigned long factory_id_;
  unsigned long map_point_id_;

  Mat descriptor_;

  Vector3d world_coord_; // coordinates in world

  list<shared_ptr<Frame>> observed_frames_; // frames that observed this point
};

} // namespace NaiveSLAM