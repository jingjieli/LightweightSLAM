#pragma once

#include "common_include.h"

namespace NaiveSLAM
{

class FeatureTracker;
class Viewer;

class System
{
public:
  enum class STATE
  {
    INITIALIZATION,
    TRACKING,
    LOST
  };

  System();
  ~System();
  System(const System &system) = delete;
  System &operator=(const System &system) = delete;
  System &operator=(const System &&system) = delete;

  void processImage(const Mat &color_img, const Mat &depth_img);

private:

  int lost_times_;
  int max_lost_times_;

  STATE system_state_;

  unique_ptr<FeatureTracker> feature_tracker_;
  unique_ptr<Viewer> viewer_;
};

} // namespace NaiveSLAM