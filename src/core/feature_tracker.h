#pragma once

#include "common_include.h"

namespace NaiveSLAM
{

class Frame;
class MapPoint;
class Map;
class PinholeCameraModel;
class Viewer;

class FeatureTracker
{
public:
  FeatureTracker();
  ~FeatureTracker();

  void createNewFrame(const SE3 &T_c_w);

  void setCurrentImages(const Mat &color_img, const Mat &depth_img);
  void extractFeatures();
  void matchFeatures();
  void estimatePose();
  void addKeyFrame();
  void updateLocalMap();

  SE3 getReferenceTransform() const;

  // check if a map point visible in this frame
  bool checkMapPointInFrame(const Vector3d &p_w, const SE3 &T_c_w) const; 

  bool verifyEstimatedPose() const;

  void updateCurrentFramePose();

  bool needNewKeyFrame() const;

  void saveCurrentFramePose();

  void setViewer(Viewer* viewer);
  void updateViewer();

private:
  Ptr<ORB> orb_detector_;
  FlannBasedMatcher flann_matcher_;
  shared_ptr<Frame> ref_frame_;
  shared_ptr<Frame> curr_frame_;
  vector<KeyPoint> curr_keypoints_;
  Mat curr_descriptors_;
  unique_ptr<Map> local_map_;

  vector<shared_ptr<MapPoint>> matched_map_pts; // matched 3d points
  vector<int> matched_pts_idx; // matched pixel

  Mat curr_color_img_;
  Mat curr_depth_img_;

  SE3 T_c_w_pnp_; // estimation from PnP
  SE3 T_c_w_estimated_; // estimation after pose optimizaiton

  unique_ptr<PinholeCameraModel> camera_model_;

  int num_inliers_;
  int min_inliers_;

  double min_rot_diff_;
  double min_trans_diff_;

  Viewer* viewer_;

  double findDepthForPixel(const KeyPoint &point);
  void addMapPoints();

  void optimizePoseG2O(const vector<Point3f> &pts_3d, const vector<Point2f> &pts_2d, const Mat &inliers);
  void optimizePoseCeres(const vector<Point3f> &pts_3d, const vector<Point2f> &pts_2d, const Mat &inliers);
};

} // namespace NaiveSLAM