#include "frame.h"
#include "map_point.h"

namespace NaiveSLAM
{

unsigned long Frame::factory_id_ = 0;

Frame::Frame()
    : frame_id_(factory_id_++)
{
}

Frame::Frame(const SE3 &T_c_w)
    : frame_id_(factory_id_++), T_c_w_(T_c_w)
{
}

Frame::~Frame()
{
}

unique_ptr<Frame> Frame::createFrame(const SE3 &T_c_w)
{
  return unique_ptr<Frame>(new Frame(T_c_w));
}

unsigned long Frame::getFrameId() const
{
  return frame_id_;
}

SE3 Frame::getTransform() const
{
  return T_c_w_;
}

void Frame::setTransform(const SE3 &new_T_c_w)
{
  T_c_w_ = new_T_c_w;
}

Vector3d Frame::getCamCenter() const
{
  return T_c_w_.inverse().translation();
}

} // namespace NaiveSLAM