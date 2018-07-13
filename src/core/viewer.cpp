#include "viewer.h"
//#include "map.h"

namespace NaiveSLAM
{

Viewer::Viewer()
{
}

Viewer::~Viewer()
{
}

void Viewer::init()
{
  rendering_thread_ = thread(&Viewer::start, this);
}

void Viewer::start()
{
  pangolin::CreateWindowAndBind("Viewer", 1024, 768);

  // 3D Mouse handler requires depth testing to be enabled
  glEnable(GL_DEPTH_TEST);

  // Issue specific OpenGl we might need
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
  pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", false, true);
  pangolin::Var<bool> menuShowKeyFrame("menu.Show KeyFrame", true, true);
  pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
  pangolin::Var<bool> menuShowPath("menu.Show Path", true, true);

  // Define Camera Render Object (for view / scene browsing)
  pangolin::OpenGlRenderState s_cam(
      pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));

  // Add named OpenGL viewport to window and provide 3D Handler
  pangolin::View &d_cam = pangolin::CreateDisplay()
                              .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)
                              .SetHandler(new pangolin::Handler3D(s_cam));

  T_w_c_.SetIdentity();

  bool is_followed = true;

  cout << "viewer start loop\n";

  while (!pangolin::ShouldQuit())
  {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (menuFollowCamera && is_followed)
    {
      s_cam.Follow(T_w_c_);
    }
    else if (menuFollowCamera && !is_followed)
    {
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0, -0.7, -1.8, 0, 0, 0, 0.0, -1.0, 0.0));
      s_cam.Follow(T_w_c_);
      is_followed = true;
    }
    else if (!menuFollowCamera && is_followed)
    {
      is_followed = false;
    }

    d_cam.Activate(s_cam);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    drawCurrentCamera();

    if (menuShowKeyFrame)
    {
      drawKeyFrames();
    }

    if (menuShowPoints)
    {
      drawMapPoints();
    }

    if (menuShowPath)
    {
      drawTrajectory();
    }

    pangolin::FinishFrame();

    this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  cout << "Viewer stops\n";
}

void Viewer::quit()
{
  rendering_thread_.join();
}

void Viewer::updateCurrentCameraPose(const SE3 &sophus_T_c_w)
{
  unique_lock<mutex> lock(render_mtx_);

  auto sophus_T_w_c = sophus_T_c_w.inverse();

  poseToGlMatrix(sophus_T_w_c, T_w_c_);
}

void Viewer::updateMapPoints(const vector<Vector3d> &points)
{
  unique_lock<mutex> lock(render_mtx_);
  map_points_ = points;
}

void Viewer::updateKeyFrames(const vector<SE3> &poses)
{
  unique_lock<mutex> lock(render_mtx_);
  keyframe_poses_ = poses;
}

void Viewer::updateTrajectory(const vector<SE3> &all_frame_poses)
{
  unique_lock<mutex> lock(render_mtx_);
  all_frame_poses_ = all_frame_poses;
}

void Viewer::drawCurrentCamera()
{
  unique_lock<mutex> lock(render_mtx_);

  const float &w = 0.08f;
  const float h = w * 0.75;
  const float z = w * 0.6;

  glPushMatrix();

  glMultMatrixd(T_w_c_.m);

  glLineWidth(3);
  glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(w, h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, -h, z);
  glVertex3f(0, 0, 0);
  glVertex3f(-w, h, z);

  glVertex3f(w, h, z);
  glVertex3f(w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(-w, -h, z);

  glVertex3f(-w, h, z);
  glVertex3f(w, h, z);

  glVertex3f(-w, -h, z);
  glVertex3f(w, -h, z);

  glEnd();
  glPopMatrix();
}

void Viewer::drawKeyFrames()
{
  unique_lock<mutex> lock(render_mtx_);

  if (keyframe_poses_.empty())
    return;

  const float &w = 0.05f;
  const float h = w * 0.75;
  const float z = w * 0.6;

  for (auto pose : keyframe_poses_)
  {
    pangolin::OpenGlMatrix gl_pose;
    poseToGlMatrix(pose.inverse(), gl_pose);

    glPushMatrix();

    glMultMatrixd(gl_pose.m);

    glLineWidth(1);
    glColor3f(0.0f, 0.0f, 1.0f);
    glBegin(GL_LINES);
    glVertex3f(0, 0, 0);
    glVertex3f(w, h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, -h, z);
    glVertex3f(0, 0, 0);
    glVertex3f(-w, h, z);

    glVertex3f(w, h, z);
    glVertex3f(w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(-w, -h, z);

    glVertex3f(-w, h, z);
    glVertex3f(w, h, z);

    glVertex3f(-w, -h, z);
    glVertex3f(w, -h, z);
    glEnd();

    glPopMatrix();
  }
}

void Viewer::drawMapPoints()
{
  unique_lock<mutex> lock(render_mtx_);

  if (map_points_.empty())
    return;

  glPointSize(2);
  glBegin(GL_POINTS);
  glColor3f(1.0, 0.0, 0.0);

  for (auto point : map_points_)
  {
    glVertex3f(point.x(), point.y(), point.z());
  }

  glEnd();
}

void Viewer::drawTrajectory()
{
  if (all_frame_poses_.empty())
    return;

  unique_lock<mutex> lock(render_mtx_);

  glLineWidth(2);
  glColor3f(0.0f, 1.0f, 0.0f);
  glBegin(GL_LINES);

  for (size_t i = 0; i < all_frame_poses_.size() - 1; ++i)
  {
    auto pose_1 = all_frame_poses_[i].inverse();
    auto pose_2 = all_frame_poses_[i + 1].inverse();
    glVertex3f(
        (float)pose_1.translation().x(),
        (float)pose_1.translation().y(),
        (float)pose_1.translation().z());

    glVertex3f(
        (float)pose_2.translation().x(),
        (float)pose_2.translation().y(),
        (float)pose_2.translation().z());
  }

  glEnd();
}

void Viewer::poseToGlMatrix(const SE3 &sophus_pose, pangolin::OpenGlMatrix &gl_pose)
{
  Matrix3d R = sophus_pose.rotation_matrix();
  Vector3d t = sophus_pose.translation();

  gl_pose.m[0] = R(0, 0);
  gl_pose.m[1] = R(1, 0);
  gl_pose.m[2] = R(2, 0);
  gl_pose.m[3] = 0.0;

  gl_pose.m[4] = R(0, 1);
  gl_pose.m[5] = R(1, 1);
  gl_pose.m[6] = R(2, 1);
  gl_pose.m[7] = 0.0;

  gl_pose.m[8] = R(0, 2);
  gl_pose.m[9] = R(1, 2);
  gl_pose.m[10] = R(2, 2);
  gl_pose.m[11] = 0.0;

  gl_pose.m[12] = t.x();
  gl_pose.m[13] = t.y();
  gl_pose.m[14] = t.z();
  gl_pose.m[15] = 1.0;
}

} // namespace NaiveSLAM