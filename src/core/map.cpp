#include "map.h"
#include "frame.h"
#include "map_point.h"

namespace NaiveSLAM
{

Map::Map()
{
}

Map::~Map()
{
}

void Map::insertKeyFrame(shared_ptr<Frame> frame)
{
  auto frame_id = frame->getFrameId();
  if (keyframes_.find(frame_id) == keyframes_.end())
  {
    keyframes_.insert(make_pair(frame_id, frame));
  }
  else
  {
    keyframes_[frame_id] = frame;
  }
}

void Map::insertMapPoint(shared_ptr<MapPoint> map_point)
{
  auto map_point_id = map_point->getMapPointId();
  if (map_points_.find(map_point_id) == map_points_.end())
  {
    map_points_.insert(make_pair(map_point_id, map_point));
  }
  else
  {
    map_points_[map_point_id] = map_point;
  }
}

size_t Map::getKeyFramesNumber() const
{
  return keyframes_.size();
}

unordered_map<unsigned long, shared_ptr<Frame>>& Map::getKeyframes()
{
  return keyframes_;
}

unordered_map<unsigned long, shared_ptr<MapPoint>>& Map::getMapPoints()
{
  return map_points_;
}

} // namespace NaiveSLAM