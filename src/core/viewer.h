#pragma once

#include "common_include.h"

namespace NaiveSLAM
{

class Viewer
{
public:
  Viewer();
  ~Viewer();
  void init();
  void start();
  void quit();

  void updateCurrentCameraPose(const SE3 &sophus_T_c_w);
  void updateMapPoints(const vector<Vector3d> &points, RenderMode mode);
  void updateKeyFrames(const vector<SE3> &poses);
  void updateTrajectory(const vector<SE3> &all_frame_poses);

private:
  thread rendering_thread_;
  pangolin::OpenGlMatrix T_w_c_;
  vector<Vector3d> map_points_;
  vector<Vector3d> all_map_points_;
  vector<SE3> keyframe_poses_;
  vector<SE3> all_frame_poses_;

  mutex render_mtx_;

  void drawCurrentCamera();
  void drawKeyFrames();
  void drawMapPoints(RenderMode mode);
  void drawTrajectory();
  void poseToGlMatrix(const SE3 &sophus_pose, pangolin::OpenGlMatrix &gl_pose);
};

} // namespace NaiveSLAM