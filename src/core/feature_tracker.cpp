#include "feature_tracker.h"
#include "frame.h"
#include "map_point.h"
#include "map.h"
#include "pinhole_camera_model.h"
#include "g2o_solvers.h"
#include "ceres_problems.h"
#include "viewer.h"

namespace NaiveSLAM
{

FeatureTracker::FeatureTracker()
    : num_inliers_(0), min_inliers_(20),
      min_rot_diff_(0.1), min_trans_diff_(0.1),
      flann_matcher_(new flann::LshIndexParams(5, 10, 2)),
      camera_model_(new PinholeCameraModel(550.083, 550.083, 319.834, 275.380)),
      local_map_(new Map())
{
  orb_detector_ = ORB::create(500, 1.2f, 8); // default values
}

FeatureTracker::~FeatureTracker()
{
}

void FeatureTracker::createNewFrame(const SE3 &T_c_w)
{
  curr_frame_ = move(Frame::createFrame(T_c_w));
}

void FeatureTracker::setCurrentImages(const Mat &color_img, const Mat &depth_img)
{
  color_img.copyTo(curr_color_img_);
  depth_img.copyTo(curr_depth_img_);
}

void FeatureTracker::extractFeatures()
{
  curr_keypoints_.clear();
  curr_descriptors_ = 0;
  orb_detector_->detectAndCompute(curr_color_img_, Mat(), curr_keypoints_, curr_descriptors_);
  cout << "extract " << curr_keypoints_.size() << " features\n";
}

void FeatureTracker::matchFeatures()
{
  vector<DMatch> matches;

  // select candidates from local map
  Mat map_pts_descriptors;
  vector<shared_ptr<MapPoint>> candidates;

  const auto &all_map_pts = local_map_->getMapPoints();

  cout << "total " << all_map_pts.size() << " map points\n";

  for (auto &map_point : all_map_pts)
  {
    auto &ptr = map_point.second;
    auto pt_in_frame = checkMapPointInFrame(ptr->getPosition(), curr_frame_->getTransform());

    if (pt_in_frame)
    {
      local_map_->updateMapPointVisibility(map_point.first);
      candidates.emplace_back(ptr);
      map_pts_descriptors.push_back(ptr->getDescriptor());
    }
  }
  cout << "use " << candidates.size() << " candidates\n";

  flann_matcher_.match(map_pts_descriptors, curr_descriptors_, matches);

  matched_map_pts.clear();
  matched_pts_idx.clear();

  if(matches.empty())
  { 
    cout << "no matches found\n";
    return;
  }
  else
  {
    cout << "flann found " << matches.size() << " matches\n";
  }

  // select best matches
  auto min_dist = min_element(
                      begin(matches), end(matches),
                      [](const DMatch &m1, const DMatch &m2) {
                        return m1.distance < m2.distance;
                      })
                      ->distance;

  for (const auto &m : matches)
  {
    if (m.distance < max(min_dist, 30.0f))
    {
      matched_map_pts.push_back(candidates[m.queryIdx]);
      matched_pts_idx.push_back(m.trainIdx);
    }
  }

  cout << "good matches: " << matched_map_pts.size() << '\n';
}

void FeatureTracker::estimatePose()
{
  vector<Point3f> pts_3d; // matched 3d points
  vector<Point2f> pts_2d; // 2d features points

  for (auto i : matched_pts_idx)
  {
    pts_2d.push_back(curr_keypoints_[i].pt);
  }

  for (const auto &p : matched_map_pts)
  {
    auto pos = p->getPosition();
    pts_3d.push_back(
        Point3f{
            (float)pos(0, 0),
            (float)pos(1, 0),
            (float)pos(2, 0)});
  }

  Mat K = camera_model_->getCameraMatrix();

  Mat rvec, tvec, inliers;
  cout << "pts_3d: " << pts_3d.size() << '\n';
  cout << "pts_2d: " << pts_2d.size() << '\n';

  if (pts_3d.size() < 4) // a precondition to call pnp solver
    return;

  solvePnPRansac(pts_3d, pts_2d, K, Mat(), rvec, tvec, false, 100, 4.0, 0.99, inliers);

  num_inliers_ = inliers.rows;
  cout << "pnp inliers: " << num_inliers_ << '\n';

  T_c_w_pnp_ = SE3(
      SO3(rvec.at<double>(0, 0), rvec.at<double>(1, 0), rvec.at<double>(2, 0)),
      Vector3d(tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0)));

  cout << "T_c_w_pnp_: " << T_c_w_pnp_.matrix() << '\n';

  // g2o 
  //optimizePoseG2O(pts_3d, pts_2d, inliers);

  // ceres-solver
  optimizePoseCeres(pts_3d, pts_2d, inliers);

  cout << "T_c_w_estimated: " << T_c_w_estimated_.matrix() << '\n';
}

void FeatureTracker::addKeyFrame()
{
  cout << "addKeyFrame\n";
  if (local_map_->getKeyFramesNumber() == 0)
  {
    for (size_t i = 0; i < curr_keypoints_.size(); ++i)
    {
      auto kp = curr_keypoints_[i];
      auto depth = findDepthForPixel(kp);

      if (depth < 0)
        continue;

      Vector3d point_world = camera_model_->pixel2world(
          Vector2d{kp.pt.x, kp.pt.y}, curr_frame_->getTransform(), depth);

      shared_ptr<MapPoint> map_point = move(MapPoint::createMapPoint(point_world, curr_descriptors_.row(i).clone(), curr_frame_));

      local_map_->addPointToMap(map_point);
      curr_frame_->addPointToFrame(map_point);
    }
    cout << "added map points to local map\n";
  }

  local_map_->addFrameToMap(curr_frame_);
  ref_frame_ = curr_frame_;

  cout << "new KeyFrame added\n";
}

void FeatureTracker::addMapPoints()
{
  vector<bool> matched(curr_keypoints_.size(), false);
  for (auto i : matched_pts_idx)
  {
    matched[i] = true;
  }

  // for those features that can't be matched to previous
  // add them to local map for later use
  for (size_t i = 0; i < curr_keypoints_.size(); ++i)
  {
    if (matched[i])
      continue;

    auto kp = curr_keypoints_[i];
    double depth = findDepthForPixel(kp);
    if (depth < 0)
    {
      matched[i] = true; // dirty way to mark a point unused
      continue;
    }

    Vector3d point_world = camera_model_->pixel2world(
        Vector2d{kp.pt.x, kp.pt.y}, curr_frame_->getTransform(), depth);

    shared_ptr<MapPoint> map_point = move(MapPoint::createMapPoint(point_world, curr_descriptors_.row(i).clone(), curr_frame_));

    local_map_->addPointToMap(map_point);
    curr_frame_->addPointToFrame(map_point);
  }

  cout << "added " << count(matched.begin(), matched.end(), false) << " map points\n";
}

void FeatureTracker::updateLocalMap()
{
  const auto &local_map_points = local_map_->getMapPoints();

  cout << "map points: " << local_map_points.size() << " before update\n";

  vector<unsigned long> pt_ids;

  for (auto iter = local_map_points.begin(); iter != local_map_points.end(); ++iter)
  {
    if (!checkMapPointInFrame(iter->second->getPosition(), curr_frame_->getTransform()))
    {
      pt_ids.push_back(iter->first);
    }
  }

  for (auto id : pt_ids)
  {
    // add it to global map before remove from local map
    local_map_->addPointToGlobalMap(id);
    local_map_->removeMapPoint(id);
  }

  cout << "map points: " << local_map_points.size() << " after update\n";

  if (matched_pts_idx.size() < 100)
  {
    addMapPoints();
  }
}

double FeatureTracker::findDepthForPixel(const KeyPoint &point)
{
  auto x = cvRound(point.pt.x);
  auto y = cvRound(point.pt.y);
  auto depth = curr_depth_img_.ptr<ushort>(y)[x];

  if (depth != 0)
  {
    return double(depth) / 1000.0; // convert from mm to meter
  }
  else
  {
    // check neighbour points
    int dx[4]{-1, 0, 1, 0};
    int dy[4]{0, -1, 0, 1};

    for (unsigned int i = 0; i < 4; ++i)
    {
      auto d = curr_depth_img_.ptr<ushort>(y + dy[i])[x + dx[i]];

      if (d != 0)
        return double(d) / 1000.0;
    }
  }

  return -1.0;
}

SE3 FeatureTracker::getReferenceTransform() const
{
  return ref_frame_->getTransform();
}

bool FeatureTracker::checkMapPointInFrame(const Vector3d &p_w, const SE3 &T_c_w) const
{
  auto p_c = camera_model_->world2camera(p_w, T_c_w);

  if (p_c(2, 0) < 0)
    return false;

  auto p_p = camera_model_->world2pixel(p_w, T_c_w);

  return p_p(0, 0) > 0 && p_p(1, 0) > 0 &&
         p_p(0, 0) < curr_color_img_.cols && p_p(1, 0) < curr_color_img_.rows;
}

bool FeatureTracker::verifyEstimatedPose() const
{
  if (num_inliers_ < min_inliers_)
  {
    cout << "num_inliers: " << num_inliers_ << " not enough\n";
    return false;
  }

  SE3 T_r_c = ref_frame_->getTransform() * T_c_w_estimated_.inverse(); // curr_cam -> ref_cam
  Sophus::Vector6d d = T_r_c.log();
  if (d.norm() > 5.0)
  {
    cout << "large motion between frames\n";
    return false;
  }

  return true;
}

void FeatureTracker::updateCurrentFramePose()
{
  curr_frame_->setTransform(T_c_w_estimated_);
}

bool FeatureTracker::needNewKeyFrame() const
{
  SE3 T_r_c = ref_frame_->getTransform() * T_c_w_estimated_.inverse(); // curr_cam -> ref_cam
  Sophus::Vector6d d = T_r_c.log();
  Vector3d trans = d.head<3>();
  Vector3d rot = d.tail<3>();
  if (rot.norm() > min_rot_diff_ || trans.norm() > min_trans_diff_)
    return true;

  return false;
}

void FeatureTracker::saveCurrentFramePose()
{
  local_map_->saveFramePose(curr_frame_->getTransform());
}

void FeatureTracker::setViewer(Viewer* viewer)
{
  viewer_ = viewer;
}

void FeatureTracker::updateViewer()
{
  viewer_->updateCurrentCameraPose(curr_frame_->getTransform());
  viewer_->updateMapPoints(local_map_->getMapPointsToRender(RenderMode::LOCAL), RenderMode::LOCAL);
  viewer_->updateMapPoints(local_map_->getMapPointsToRender(RenderMode::GLOBAL), RenderMode::GLOBAL);
  viewer_->updateKeyFrames(local_map_->getKeyFramesToRender());
  viewer_->updateTrajectory(local_map_->getTrajectoryToRender());
}

void FeatureTracker::optimizePoseG2O(const vector<Point3f> &pts_3d, const vector<Point2f> &pts_2d, const Mat &inliers)
{
  using BlockSolver_6_2 = BlockSolver<BlockSolverTraits<6, 2>>;
  BlockSolver_6_2::LinearSolverType *linear_solver = new LinearSolverDense<BlockSolver_6_2::PoseMatrixType>();
  BlockSolver_6_2 *solver_ptr = new BlockSolver_6_2(linear_solver);
  OptimizationAlgorithmLevenberg *algo = new OptimizationAlgorithmLevenberg(solver_ptr);
  SparseOptimizer optimizer;
  optimizer.setAlgorithm(algo);
  optimizer.setVerbose(true);

  VertexSE3Expmap *pose = new VertexSE3Expmap();
  pose->setId(0);
  pose->setEstimate(SE3Quat(
      T_c_w_pnp_.rotation_matrix(), T_c_w_pnp_.translation()));
  optimizer.addVertex(pose);

  for (int i = 0; i < inliers.rows; ++i)
  {
    auto index = inliers.at<int>(i, 0);
    EdgeProjectXYZ2UVPoseOnly *edge = new EdgeProjectXYZ2UVPoseOnly();
    edge->setId(i);
    edge->setVertex(0, pose);
    edge->camera_model_ = camera_model_.get();
    edge->point_ = Vector3d{pts_3d[index].x, pts_3d[index].y, pts_3d[index].z};
    edge->setMeasurement(Vector2d{pts_2d[index].x, pts_2d[index].y});
    edge->setInformation(Matrix2d::Identity());
    optimizer.addEdge(edge);

    local_map_->updateMapPointMatchedTimes(matched_map_pts[index]->getMapPointId());
  }

  optimizer.initializeOptimization();
  optimizer.optimize(10);

  T_c_w_estimated_ = SE3(
      pose->estimate().rotation(),
      pose->estimate().translation());
}

void FeatureTracker::optimizePoseCeres(const vector<Point3f> &pts_3d, const vector<Point2f> &pts_2d, const Mat &inliers)
{
  // Eigen: quaternion constructor order: w, x, y, z
  // but in memory it's stored as: x, y, z, w
  auto quat = Quaterniond(T_c_w_pnp_.rotation_matrix());
  auto trans = T_c_w_pnp_.translation();
  // cout << "quat(xyzw): " << quat.coeffs() << '\n';
  // cout << "quat(x,y,z,w): " << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w() << '\n';
  double initial_pose[7] {quat.x(), quat.y(), quat.z(), quat.w(), trans.x(), trans.y(), trans.z()};

  ceres::Problem problem;
  for (int i = 0; i < inliers.rows; ++i)
  {
    auto index = inliers.at<int>(i, 0);

    problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7>(
        new ReprojectionError(
          Vector3d{pts_3d[index].x, pts_3d[index].y, pts_3d[index].z},
          Vector2d{pts_2d[index].x, pts_2d[index].y},
          camera_model_.get())
      ),
      nullptr,
      initial_pose
    );
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_SCHUR;
  options.minimizer_progress_to_stdout = true;

  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  cout << summary.FullReport() << '\n';

  Quaterniond quat_est(initial_pose[3], initial_pose[0], initial_pose[1], initial_pose[2]);
  Matrix3d R_est(quat_est);
  Vector3d t_est(initial_pose[4], initial_pose[5], initial_pose[6]);

  T_c_w_estimated_ = SE3(R_est, t_est);
}

} // namespace NaiveSLAM