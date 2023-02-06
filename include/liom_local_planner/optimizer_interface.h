//
// Created by yenkn on 1/11/23.
//

#ifndef SRC_OPTIMIZER_INTERFACE_H
#define SRC_OPTIMIZER_INTERFACE_H
#include <vector>
#include <array>
#include <tuple>
#include <limits>
#include <memory>
#include <Eigen/Dense>

#include "math/pose.h"
#include "planner_config.h"
#include "environment.h"

namespace liom_local_planner {

constexpr int NVar = 7;

using TrajectoryPointVector = Eigen::Matrix<double, NVar, 1>;

struct TrajectoryPoint {
  double x = 0.0, y = 0.0, theta = 0.0, v = 0.0, phi = 0.0, a = 0.0, omega = 0.0;

  TrajectoryPoint() = default;
  explicit TrajectoryPoint(const Eigen::Ref<const TrajectoryPointVector>& vec)
    : x(vec(0)), y(vec(1)), theta(vec(2)), v(vec(3)), phi(vec(4)), a(vec(5)), omega(vec(6)) {}

  inline math::Pose pose() const {
    return {x, y, theta};
  }

  inline math::Vec2d position() const {
    return {x, y};
  }

  inline TrajectoryPointVector vec() const {
    TrajectoryPointVector vec;
    vec << x, y, theta, v, phi, a, omega;
    return vec;
  }
};

using Trajectory = std::vector<TrajectoryPoint>;

struct FullStates {
  double tf;
  Trajectory states;
};

struct Constraints {
  TrajectoryPoint start, goal;
  Eigen::MatrixXd corridor_lb, corridor_ub; // nfe * ndisc
};

constexpr double inf = std::numeric_limits<double>::max();

class IOptimizer {
public:
  IOptimizer(std::shared_ptr<PlannerConfig> config, std::shared_ptr<Environment> env): config_(std::move(config)), env_(std::move(env)) {}

  virtual bool Solve(double w_inf, const Constraints &profile, const FullStates &guess, FullStates &result, double &infeasibility) {
    return false;
  }

protected:
  std::shared_ptr<PlannerConfig> config_;
  std::shared_ptr<Environment> env_;

};

}

#endif //SRC_OPTIMIZER_INTERFACE_H
