#pragma once

#include "common_include.h"

namespace NaiveSLAM
{

class Frame;
class MapPoint;

class Map
{
public:
  Map();
  ~Map();

  void insertKeyFrame(shared_ptr<Frame> keyframe);
  void insertMapPoint(shared_ptr<MapPoint> map_point);
  size_t getKeyFramesNumber() const;

  unordered_map<unsigned long, shared_ptr<Frame>>& getKeyframes();
  unordered_map<unsigned long, shared_ptr<MapPoint>>& getMapPoints();

private:
  unordered_map<unsigned long, shared_ptr<Frame>> keyframes_;
  unordered_map<unsigned long, shared_ptr<MapPoint>> map_points_;
};

} // namespace NaiveSLAM