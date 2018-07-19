#include "system.h"
#include "frame.h"
#include "feature_tracker.h"
#include "viewer.h"

namespace NaiveSLAM
{

System::System()
    : lost_times_(0), max_lost_times_(30),
      system_state_(STATE::INITIALIZATION),
      feature_tracker_(new FeatureTracker()),
      viewer_(new Viewer())
{
  feature_tracker_->setViewer(viewer_.get());
  viewer_->init();
}

System::~System()
{
}

void System::processImage(const Mat &color_img, const Mat &depth_img)
{
  feature_tracker_->setCurrentImages(color_img, depth_img);

  switch (system_state_)
  {

  case STATE::INITIALIZATION:
  {
    feature_tracker_->createNewFrame(SE3());
    feature_tracker_->extractFeatures();
    feature_tracker_->addKeyFrame();
    feature_tracker_->updateViewer();
    system_state_ = STATE::TRACKING;
    cout << "INITIALIZATION done\n";
    break;
  }

  case STATE::TRACKING:
  {
    feature_tracker_->createNewFrame(feature_tracker_->getReferenceTransform());
    feature_tracker_->extractFeatures();
    feature_tracker_->matchFeatures();
    feature_tracker_->estimatePose();

    if (feature_tracker_->verifyEstimatedPose())
    {
      feature_tracker_->updateCurrentFramePose();
      lost_times_ = 0;

      feature_tracker_->updateLocalMap();

      if (feature_tracker_->needNewKeyFrame())
      {
        feature_tracker_->addKeyFrame();
      }
      else 
      {
        // for non keyframe, only save frame pose
        feature_tracker_->saveCurrentFramePose();
      }
    }
    else
    {
      lost_times_++;
      if (lost_times_ > max_lost_times_)
        system_state_ = STATE::LOST;
    }

    feature_tracker_->updateViewer();
    break;
  }

  case STATE::LOST:
  {
    cout << "tracking lost ...\n";
    break;
  }
  }
}

} // namespace NaiveSLAM