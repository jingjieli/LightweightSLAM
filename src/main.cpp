#include "astra_rgbd_camera.h"
#include "openni2_camera.h"

using namespace std;
using namespace cv;

int main(int argc, char** argv) 
{
  unique_ptr<OpenNI2Camera> rgbd_camera = unique_ptr<OpenNI2Camera>(new OpenNI2Camera());

  if (!rgbd_camera->initialize())
    return -1;

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

    imshow("color frame", color_frame);
    imshow("depth frame", vis_depth);

    auto key = cv::waitKey(1);

    if (key == 'q')
      break;
  }

  return 0;
}