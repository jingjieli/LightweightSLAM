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

void Map::addFrameToMap(shared_ptr<Frame> frame)
{
  unique_lock<mutex> lock(data_mtx_);
  auto frame_id = frame->getFrameId();
  all_frame_poses_.push_back(frame->getTransform());
  if (keyframes_.find(frame_id) == keyframes_.end())
  {
    keyframes_.insert(make_pair(frame_id, frame));
  }
  else
  {
    keyframes_[frame_id] = frame;
  }
}

void Map::addPointToMap(shared_ptr<MapPoint> map_point)
{
  unique_lock<mutex> lock(data_mtx_);
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

void Map::addPointToGlobalMap(unsigned long pt_id)
{
  unique_lock<mutex> lock(data_mtx_);
  
  auto iter = map_points_.find(pt_id);

  if (iter != map_points_.end())
  {
    all_map_points_.insert(make_pair(pt_id, iter->second));
  }
}

size_t Map::getKeyFramesNumber() const
{
  return keyframes_.size();
}

const Map::KEYFRAMES &Map::getKeyframes()
{
  unique_lock<mutex> lock(data_mtx_);
  return keyframes_;
}

const Map::MAP_POINTS &Map::getMapPoints()
{
  unique_lock<mutex> lock(data_mtx_);
  return map_points_;
}

const vector<Vector3d> &Map::getMapPointsToRender(RenderMode mode)
{
  switch (mode)
  {
    case RenderMode::LOCAL:
    {
      map_points_coord.clear();
      unique_lock<mutex> lock(data_mtx_);
      for (auto iter = map_points_.begin(); iter != map_points_.end(); ++iter)
      {
        map_points_coord.push_back(iter->second->getPosition());
      }
      return map_points_coord;
    }
    case RenderMode::GLOBAL:
    {
      all_map_points_coord.clear();
      unique_lock<mutex> lock(data_mtx_);
      for (auto iter = all_map_points_.begin(); iter != all_map_points_.end(); ++iter)
      {
        all_map_points_coord.push_back(iter->second->getPosition());
      }
      return all_map_points_coord;
    }
  }
}

const vector<SE3> &Map::getKeyFramesToRender()
{
  keyframe_poses_.clear();
  unique_lock<mutex> lock(data_mtx_);
  for (auto iter = keyframes_.begin(); iter != keyframes_.end(); ++iter)
  {
    keyframe_poses_.push_back(iter->second->getTransform());
  }
  return keyframe_poses_;
}

const vector<SE3> &Map::getTrajectoryToRender()
{
  unique_lock<mutex> lock(data_mtx_);
  return all_frame_poses_;
}

void Map::updateMapPointVisibility(unsigned long pt_id)
{
  unique_lock<mutex> lock(data_mtx_);
  auto iter = map_points_.find(pt_id);

  if (iter != map_points_.end())
  {
    iter->second->visible_times_++;
  }
}

void Map::updateMapPointMatchedTimes(unsigned long pt_id)
{
  unique_lock<mutex> lock(data_mtx_);
  auto iter = map_points_.find(pt_id);

  if (iter != map_points_.end())
  {
    iter->second->matched_times_++;
  }
}

void Map::removeMapPoint(unsigned long pt_id)
{
  unique_lock<mutex> lock(data_mtx_);
  auto iter = map_points_.find(pt_id);

  if (iter != map_points_.end())
  {
    map_points_.erase(iter);
  }
}

void Map::saveFramePose(const SE3 &pose)
{
  unique_lock<mutex> lock(data_mtx_);
  all_frame_poses_.push_back(pose);
}

} // namespace NaiveSLAM