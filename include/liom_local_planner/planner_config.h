#pragma once
#include "vehicle_model.h"

namespace liom_local_planner {

struct PlannerConfig {
   double xy_resolution = 0.2;
   double theta_resolution = 0.1;
   double step_size = 0.3;
   int next_node_num = 6;
   double grid_xy_resolution = 0.5;
   double forward_penalty = 0.5;
   double backward_penalty = 1.0;
   double gear_change_penalty = 5.0;
   double steering_penalty = 0.1;
   double steering_change_penalty = 1.0;

  /**
   * Minimum number of finite elements used to discretize an OCP
   */
  int min_nfe = 20;

  /**
   *
   */
  double time_step = 0.4;

  /**
   * maximum iteration count for corridor expansion
   */
  int corridor_max_iter = 1000;

  /**
   * increment limit for corridor expansion
   */
  double corridor_incremental_limit = 20.0;

  /**
   * Weighting parameter for control input acceleration
   */
  double opti_w_a = 1.0;

  /**
   * weighting parameter for control input omega
   */
  double opti_w_omega = 1.0;

  int opti_inner_iter_max = 100;

  /**
   * Initial value of weighting parameter w_penalty
   */
  double opti_w_penalty0 = 1e4;

  /**
   * Violation tolerance w.r.t. the softened nonlinear constraints
   */
  double opti_varepsilon_tol = 1e-4;

  VehicleModel vehicle;
};

}