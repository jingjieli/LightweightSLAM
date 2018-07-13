#pragma once

#include "common_include.h"

namespace NaiveSLAM
{

class Frame;
class MapPoint;

class Map
{
  using LOCAL_KEYFRAMES = unordered_map<unsigned long, unique_ptr<Frame>>;
  using LOCAL_MAP_POINTS = unordered_map<unsigned long, unique_ptr<MapPoint>>;

public:
  Map();
  ~Map();
  Map(const Map &map) = delete;
  Map(Map &&map) = delete;
  Map &operator=(const Map &map) = delete;
  Map &operator=(Map &&map) = delete;

  void createNewFrame(Frame *&frame_ptr, const SE3 &T_c_w);

  void getCurrentFrame(Frame *&frame_ptr) const;

  void insertCurrentFrame();
  void insertMapPoint(const Vector3d &world_coord, const Mat &descriptor, Frame *frame);
  size_t getKeyFramesNumber() const;

  const LOCAL_KEYFRAMES &getKeyframes();
  const LOCAL_MAP_POINTS &getMapPoints();

  const vector<Vector3d> &getMapPointsToRender();
  const vector<SE3> &getKeyFramesToRender();
  const vector<SE3> &getTrajectoryToRender();

  void updateMapPointVisibility(unsigned long pt_id);
  void updateMapPointMatchedTimes(unsigned long pt_id);
  void removeMapPoint(unsigned long pt_id);

  SE3 getCurrentFrameTransform();
  void updateCurrentFrameTransform(const SE3 &pose);

  void finalizeCurrentFrame();

private:
  LOCAL_KEYFRAMES keyframes_;
  LOCAL_MAP_POINTS map_points_;

  unique_ptr<Frame> curr_frame_;

  mutex data_mtx_;

  // ****** for render ******
  vector<Vector3d> map_points_coord;
  vector<SE3> keyframe_poses_;
  vector<SE3> all_frame_poses_;

  unique_ptr<MapPoint> createNewMapPoint(const Vector3d &world_coord, const Mat &descriptor, Frame *frame);
};

} // namespace NaiveSLAM