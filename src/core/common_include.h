#pragma once

#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <algorithm>

#include "sophus/se3.h"
#include "sophus/so3.h"

#include <Eigen/Core>

#include "opencv2/opencv.hpp"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/local_parameterization.h>

#include "pangolin/pangolin.h"

using namespace std;
using namespace Sophus;
using namespace Eigen;
using namespace cv;
using namespace g2o;

namespace NaiveSLAM
{
enum class RenderMode
{
  LOCAL = 0,
  GLOBAL = 1
};

enum class ErrorTerm
{
  PHOTOMETRIC = 0,
  REPROJECTION = 1
};
} // namespace NaiveSLAM