#pragma once

#include "common_include.h"

namespace NaiveSLAM
{

class Frame;
class MapPoint;
class Map;
class PinholeCameraModel;

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

  void updateCurrentFrameTransform();

  bool needNewKeyFrame() const;

private:
  Ptr<ORB> orb_detector_;
  FlannBasedMatcher flann_matcher_;
  Frame* ref_frame_ = nullptr;
  Frame* curr_frame_ = nullptr;
  vector<KeyPoint> curr_keypoints_;
  Mat curr_descriptors_;
  unique_ptr<Map> local_map_;

  vector<MapPoint*> matched_map_pts; // matched 3d points
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

  double findDepthForPixel(const KeyPoint &point);
  void addMapPoints();
};

} // namespace NaiveSLAM