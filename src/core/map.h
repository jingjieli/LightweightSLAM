#pragma once

#include "common_include.h"

namespace NaiveSLAM
{

class Frame;
class MapPoint;

class Map
{
  using KEYFRAMES = unordered_map<unsigned long, shared_ptr<Frame>>;
  using MAP_POINTS = unordered_map<unsigned long, shared_ptr<MapPoint>>;

public:
  Map();
  ~Map();
  Map(const Map &map) = delete;
  Map(Map &&map) = delete;
  Map &operator=(const Map &map) = delete;
  Map &operator=(Map &&map) = delete;

  void addFrameToMap(shared_ptr<Frame> frame);
  void addPointToMap(shared_ptr<MapPoint> map_point);
  void addPointToGlobalMap(unsigned long id);
  size_t getKeyFramesNumber() const;

  const KEYFRAMES &getKeyframes();
  const MAP_POINTS &getMapPoints();

  const vector<Vector3d> &getMapPointsToRender(RenderMode mode);
  const vector<SE3> &getKeyFramesToRender();
  const vector<SE3> &getTrajectoryToRender();

  void updateMapPointVisibility(unsigned long pt_id);
  void updateMapPointMatchedTimes(unsigned long pt_id);
  void removeMapPoint(unsigned long pt_id);

  void saveFramePose(const SE3 &pose);

private:
  KEYFRAMES keyframes_;
  MAP_POINTS map_points_;
  MAP_POINTS all_map_points_;

  mutex data_mtx_;

  // ****** for render ******
  vector<Vector3d> map_points_coord;
  vector<Vector3d> all_map_points_coord;
  vector<SE3> keyframe_poses_;
  vector<SE3> all_frame_poses_;

};

} // namespace NaiveSLAM