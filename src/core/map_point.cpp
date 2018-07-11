#include "map_point.h"
#include "frame.h"

namespace NaiveSLAM
{

unsigned long MapPoint::factory_id_ = 0;

shared_ptr<MapPoint> MapPoint::createMapPoint()
{
  return shared_ptr<MapPoint>(new MapPoint());
}

shared_ptr<MapPoint> MapPoint::createMapPoint(const Vector3d &world_coord, const Mat &descriptor, Frame *frame)
{
  return shared_ptr<MapPoint>(new MapPoint(world_coord, descriptor, frame));
}

MapPoint::MapPoint()
    : map_point_id_(factory_id_++)
{
}

MapPoint::MapPoint(const Vector3d &world_coord, const Mat &descriptor, Frame *frame)
    : map_point_id_(factory_id_++), world_coord_(world_coord), descriptor_(descriptor)
{
  observed_frames_.push_back(frame);
}

MapPoint::~MapPoint()
{
}

unsigned long MapPoint::getMapPointId() const
{
  return map_point_id_;
}

const Vector3d& MapPoint::getPosition() const
{
  return world_coord_;
}

const Mat& MapPoint::getDescriptor() const
{
  return descriptor_;
}

} // namespace NaiveSLAM