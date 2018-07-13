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

void Map::createNewFrame(Frame *&frame_ptr, const SE3 &T_c_w)
{
  curr_frame_ = Frame::createFrame(T_c_w);
  frame_ptr = curr_frame_.get();
}

unique_ptr<MapPoint> Map::createNewMapPoint(const Vector3d &world_coord, const Mat &descriptor, Frame *frame)
{
  return MapPoint::createMapPoint(world_coord, descriptor, frame);
}

void Map::getCurrentFrame(Frame *&frame_ptr) const
{
  frame_ptr = curr_frame_.get();
}

void Map::insertCurrentFrame()
{
  unique_lock<mutex> lock(data_mtx_);
  auto frame_id = curr_frame_->getFrameId();
  all_frame_poses_.push_back(curr_frame_->getTransform());
  if (keyframes_.find(frame_id) == keyframes_.end())
  {
    keyframes_.insert(make_pair(frame_id, std::move(curr_frame_)));
  }
  else
  {
    keyframes_[frame_id] = std::move(curr_frame_);
  }
}

void Map::insertMapPoint(const Vector3d &world_coord, const Mat &descriptor, Frame *frame)
{
  unique_lock<mutex> lock(data_mtx_);
  auto new_map_point = createNewMapPoint(world_coord, descriptor, frame);
  auto map_point_id = new_map_point->getMapPointId();
  if (map_points_.find(map_point_id) == map_points_.end())
  {
    map_points_.insert(make_pair(map_point_id, std::move(new_map_point)));
  }
  else
  {
    map_points_[map_point_id] = std::move(new_map_point);
  }
}

size_t Map::getKeyFramesNumber() const
{
  return keyframes_.size();
}

const Map::LOCAL_KEYFRAMES &Map::getKeyframes()
{
  unique_lock<mutex> lock(data_mtx_);
  return keyframes_;
}

const Map::LOCAL_MAP_POINTS &Map::getMapPoints()
{
  unique_lock<mutex> lock(data_mtx_);
  return map_points_;
}

const vector<Vector3d> &Map::getMapPointsToRender()
{
  map_points_coord.clear();
  unique_lock<mutex> lock(data_mtx_);
  for (auto iter = map_points_.begin(); iter != map_points_.end(); ++iter)
  {
    map_points_coord.push_back(iter->second->getPosition());
  }
  return map_points_coord;
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

SE3 Map::getCurrentFrameTransform()
{
  unique_lock<mutex> lock(data_mtx_);

  return curr_frame_ ? curr_frame_->getTransform() : SE3();
}

void Map::updateCurrentFrameTransform(const SE3 &pose)
{
  unique_lock<mutex> lock(data_mtx_);
  curr_frame_->setTransform(pose);
}

void Map::finalizeCurrentFrame()
{
  unique_lock<mutex> lock(data_mtx_);
  all_frame_poses_.push_back(curr_frame_->getTransform());
}

} // namespace NaiveSLAM