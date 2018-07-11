#include "astra_rgbd_camera.h"
#include "openni2_camera.h"
#include "system.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv) 
{
  unique_ptr<OpenNI2Camera> rgbd_camera = unique_ptr<OpenNI2Camera>(new OpenNI2Camera());

  if (!rgbd_camera->initialize())
    return -1;

  unique_ptr<NaiveSLAM::System> system = unique_ptr<NaiveSLAM::System>(
    new NaiveSLAM::System());

  int skip_count = 0;

  while (true) 
  {
    if (!rgbd_camera->update())
      return -1;

    Mat color_frame(480, 640, CV_8UC3);
    Mat depth_frame(480, 640, CV_16SC1);
    Mat vis_depth(480, 640, CV_8UC1);

    rgbd_camera->grabColorFrame(color_frame);
    rgbd_camera->grabDepthFrame(depth_frame);
    rgbd_camera->grabDepthFrameForVisualize(vis_depth);

    // cout << "Color frame: " << color_frame.cols << " " << color_frame.rows << endl;
    // cout << "Depth frame: " << depth_frame.cols << " " << depth_frame.rows << endl;

    if (color_frame.empty() || depth_frame.empty() || vis_depth.empty())
      continue;

    if (skip_count > 30)
    {
      system->processImage(color_frame, depth_frame);
    }
    else
    {
      skip_count++;
    }

    imshow("color frame", color_frame);
    imshow("depth frame", vis_depth);

    auto key = cv::waitKey(1);

    if (key == 'q')
      break;
  }

  return 0;
}