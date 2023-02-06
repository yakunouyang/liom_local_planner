#pragma once
#include <Eigen/Dense>
#include <memory>
#include <utility>
#include <coin/IpIpoptApplication.hpp>

#include "optimizer_interface.h"
#include "planner_config.h"
#include "environment.h"

namespace liom_local_planner {

using RawStates = Eigen::VectorXd;

class LightweightProblem: public IOptimizer {
public:
  LightweightProblem(std::shared_ptr<PlannerConfig> config, std::shared_ptr<Environment> env);

  bool Solve(
      double w_inf,
      const Constraints &profile,
      const FullStates &guess,
      FullStates &result,
      double &infeasibility) override;

private:
  double normalizer_tf_ = 1.0, normalizer_disc_ = 1.0;
  std::vector<double> normalizer_ = { 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0 };

  Ipopt::IpoptApplication app_;

  inline Eigen::VectorXd GetStatesNormalizerVector() const {
    Eigen::VectorXd vec(NVar + config_->vehicle.n_disc * 2);
    vec.head<NVar>() = Eigen::Map<const TrajectoryPointVector>(normalizer_.data());
    vec.tail(vec.size() - NVar).setConstant(normalizer_disc_);
    return vec;
  }

  inline FullStates ConvertVectorToStates(
      const double *x0, int nfe, const TrajectoryPoint &start, const TrajectoryPoint &goal) const {
    FullStates result;
    result.tf = x0[0] * normalizer_tf_;
    result.states.resize(nfe);
    result.states.front() = start;
    result.states.back() = goal;

    auto normalizer_vec = GetStatesNormalizerVector();

    int ncols = NVar + config_->vehicle.n_disc * 2;
    Eigen::Map<const Eigen::ArrayXXd> states(x0 + 1, nfe-2, ncols); // skip variable tf
    for(int i = 1; i < nfe - 1; i++) {
      result.states[i] = TrajectoryPoint((states.row(i-1) * normalizer_vec.array().transpose()).head<NVar>());
    }
    return result;
  }
};


}