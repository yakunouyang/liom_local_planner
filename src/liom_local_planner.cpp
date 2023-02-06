//
// Created by yenkn on 1/11/23.
//
#include <ros/ros.h>

#include "liom_local_planner/time.h"
#include "liom_local_planner/liom_local_planner.h"
#include "liom_local_planner/lightweight_nlp_problem.h"
#include "liom_local_planner/math/math_utils.h"
#include "liom_local_planner/visualization/plot.h"

namespace liom_local_planner {

LiomLocalPlanner::LiomLocalPlanner(std::shared_ptr<PlannerConfig> config, std::shared_ptr<Environment> env)
  : config_(config), env_(env), coarse_path_planner_(config, env) {
  problem_ = std::make_shared<LightweightProblem>(config_, env_);
}

FullStates LiomLocalPlanner::GenerateGuessFromPath(const std::vector<math::Pose> &path, const TrajectoryPoint &start) {
  ROS_ASSERT_MSG(!path.empty(), "global path empty");

  size_t closest_index = 0;
  double closest_dist = path.front().DistanceTo(start.position());
  for(size_t i = 1; i < path.size(); i++) {
    double dist = path[i].DistanceTo(start.position());
    if(dist < closest_dist) {
      closest_dist = dist;
      closest_index = i;
    }
  }
  std::vector<math::Pose> pruned_path;
  std::copy(std::next(path.begin(), closest_index), path.end(), std::back_inserter(pruned_path));
  return ResamplePath(pruned_path);
}

FullStates LiomLocalPlanner::StitchPreviousSolution(const FullStates &solution, const TrajectoryPoint &start) {
  FullStates result;
  if(solution.states.empty()) {
    return result;
  }

  size_t closest_index = 0;
  double closest_dist = hypot(solution.states.front().x - start.x, solution.states.front().y - start.y);
  for(size_t i = 1; i < solution.states.size(); i++) {
    double dist = hypot(solution.states[i].x - start.x, solution.states[i].y - start.y);
    if(dist < closest_dist) {
      closest_dist = dist;
      closest_index = i;
    }
  }

  double dt = solution.tf / solution.states.size();
  result.tf = solution.tf - closest_index * dt;
  std::copy(std::next(solution.states.begin(), closest_index), solution.states.end(), std::back_inserter(result.states));
  return result;
}

bool LiomLocalPlanner::CheckGuessFeasibility(const FullStates &guess) {
  if(guess.states.empty()) {
    return false;
  }

  for(auto &state: guess.states) {
    if (env_->CheckPoseCollision(0.0, state.pose())) {
      return false;
    }
  }
  return true;
}

bool LiomLocalPlanner::Plan(const FullStates &prev_sol, const TrajectoryPoint &start, const TrajectoryPoint &goal, FullStates &result) {
  FullStates guess = StitchPreviousSolution(prev_sol, start);
  if(!CheckGuessFeasibility(guess)) {
    std::vector<math::Pose> initial_path;
    double st = GetCurrentTimestamp();
    if(!coarse_path_planner_.Plan(start.pose(), goal.pose(), initial_path)) {
      ROS_ERROR("re-plan coarse path failed!");
      return false;
    }
    guess = GenerateGuessFromPath(initial_path, start);
    ROS_INFO("coarse path generation time: %f", GetCurrentTimestamp() - st);

    std::vector<double> xs, ys;
    for(auto &pose: initial_path) {
      xs.push_back(pose.x()); ys.push_back(pose.y());
    }
    visualization::Plot(xs, ys, 0.1, visualization::Color::Yellow, 1, "Coarse Path");
    visualization::Trigger();
  }

  Constraints constraints;
  constraints.start = start;
  constraints.goal = goal;

  int disc_nvar = config_->vehicle.n_disc * 2;
  constraints.corridor_lb.setConstant(guess.states.size(), disc_nvar, -inf);
  constraints.corridor_ub.setConstant(guess.states.size(), disc_nvar, inf);
  for(size_t i = 0; i < guess.states.size(); i++) {
    auto disc_pos = config_->vehicle.GetDiscPositions(guess.states[i].x, guess.states[i].y, guess.states[i].theta);

    for(int j = 0; j < config_->vehicle.n_disc; j++) {
      math::AABox2d box;
      if (!env_->GenerateCorridorBox(0.0, disc_pos[j*2], disc_pos[j*2+1], config_->vehicle.disc_radius, box)) {
        ROS_ERROR("%d th corridor box indexed at %zu generation failed!", j, i);
        return false;
      }

      auto color = visualization::Color::Green;
      color.set_alpha(0.1);
      visualization::PlotPolygon(math::Polygon2d(math::Box2d(box)), 0.05, color, i, "Corridor " + std::to_string(j));

      constraints.corridor_lb(i, j*2) = box.min_x();
      constraints.corridor_lb(i, j*2+1) = box.min_y();
      constraints.corridor_ub(i, j*2) = box.max_x();
      constraints.corridor_ub(i, j*2+1) = box.max_y();
    }
  }

  visualization::Trigger();

  double infeasibility;
  if(!problem_->Solve(config_->opti_w_penalty0, constraints, guess, result, infeasibility)) {
    ROS_ERROR("solver failed!");
    return false;
  }

  if(infeasibility > config_->opti_varepsilon_tol) {
    ROS_WARN("infeasibility = %.6f > %.6f, trajectory may not be feasible", infeasibility, config_->opti_varepsilon_tol);
  }

  return true;
}

FullStates LiomLocalPlanner::ResamplePath(const std::vector<math::Pose> &path) const {
  std::vector<int> gears(path.size());
  std::vector<double> stations(path.size(), 0);

  for(size_t i = 1; i < path.size(); i++) {
    double tracking_angle = atan2(path[i].y() - path[i-1].y(), path[i].x() - path[i-1].x());
    bool gear = std::abs(math::NormalizeAngle(tracking_angle - path[i].theta())) < M_PI_2;
    gears[i] = gear ? 1 : -1;

    stations[i] = stations[i-1] + path[i].DistanceTo(path[i-1]);
  }

  if(gears.size() > 1) {
    gears[0] = gears[1];
  }

  std::vector<double> time_profile(gears.size());
  size_t last_idx = 0;
  double start_time = 0;
  for(size_t i = 0; i < gears.size(); i++) {
    if(i == gears.size() - 1 || gears[i+1] != gears[i]) {
      std::vector<double> station_segment;
      std::copy_n(stations.begin(), i - last_idx + 1, std::back_inserter(station_segment));

      auto profile = GenerateOptimalTimeProfileSegment(station_segment, start_time);
      std::copy(profile.begin(), profile.end(), std::next(time_profile.begin(), last_idx));
      start_time = profile.back();
      last_idx = i;
    }
  }

  int nfe = std::max(config_->min_nfe, int(time_profile.back() / config_->time_step));
  auto interpolated_ticks = math::LinSpaced(time_profile.front(), time_profile.back(), nfe);

  std::vector<double> prev_x(path.size()), prev_y(path.size()), prev_theta(path.size());
  for(size_t i = 0; i < path.size(); i++) {
    prev_x[i] = path[i].x();
    prev_y[i] = path[i].y();
    prev_theta[i] = path[i].theta();
  }

  FullStates result;
  result.tf = interpolated_ticks.back();
  result.states.resize(nfe);
  auto interp_x = math::Interpolate1d(time_profile, prev_x, interpolated_ticks);
  auto interp_y = math::Interpolate1d(time_profile, prev_y, interpolated_ticks);
  auto interp_theta = math::ToContinuousAngle(math::Interpolate1d(time_profile, prev_theta, interpolated_ticks));
  for(size_t i = 0; i < nfe; i++) {
    result.states[i].x = interp_x[i];
    result.states[i].y = interp_y[i];
    result.states[i].theta = interp_theta[i];
  }

  double dt = interpolated_ticks[1] - interpolated_ticks[0];
  for(size_t i = 0; i < nfe-1; i++) {
    double tracking_angle = atan2(result.states[i+1].y - result.states[i].y, result.states[i+1].x - result.states[i].x);
    bool gear = std::abs(math::NormalizeAngle(tracking_angle - result.states[i].theta)) < M_PI_2;
    double velocity = hypot(result.states[i+1].y - result.states[i].y, result.states[i+1].x - result.states[i].x) / dt;

    result.states[i].v = std::min(config_->vehicle.max_velocity, std::max(-config_->vehicle.max_velocity, gear ? velocity : -velocity));
    result.states[i].phi = std::min(config_->vehicle.phi_max, std::max(-config_->vehicle.phi_max, atan((result.states[i+1].theta - result.states[i].theta) * config_->vehicle.wheel_base / (result.states[i].v * dt))));
  }

  for(size_t i = 0; i < nfe-1; i++) {
    result.states[i].a = std::min(config_->vehicle.max_acceleration, std::max(-config_->vehicle.max_acceleration, (result.states[i+1].v - result.states[i].v) / dt));
    result.states[i].omega = std::min(config_->vehicle.omega_max, std::max(-config_->vehicle.omega_max, (result.states[i+1].phi - result.states[i].phi) / dt));
  }
  return result;
}

std::vector<double> LiomLocalPlanner::GenerateOptimalTimeProfileSegment(const std::vector<double> &stations, double start_time) const {
  double max_accel = config_->vehicle.max_acceleration; double max_decel = -config_->vehicle.max_acceleration;
  double max_velocity = config_->vehicle.max_velocity; double min_velocity = -config_->vehicle.max_velocity;

  int accel_idx = 0, decel_idx = stations.size()-1;
  double vi = 0.0;
  std::vector<double> profile(stations.size());
  for (int i = 0; i < stations.size()-1; i++) {
    double ds = stations[i+1] - stations[i];

    profile[i] = vi;
    vi = sqrt(vi * vi + 2 * max_accel * ds);
    vi = std::min(max_velocity, std::max(min_velocity, vi));

    if(vi >= max_velocity) {
      accel_idx = i+1;
      break;
    }
  }

  vi = 0.0;
  for (int i = stations.size()-1; i > accel_idx; i--) {
    double ds = stations[i] - stations[i-1];
    profile[i] = vi;
    vi = sqrt(vi * vi - 2 * max_decel * ds);
    vi = std::min(max_velocity, std::max(min_velocity, vi));

    if(vi >= max_velocity) {
      decel_idx = i;
      break;
    }
  }

  std::fill(std::next(profile.begin(), accel_idx), std::next(profile.begin(), decel_idx), max_velocity);

  std::vector<double> time_profile(stations.size(), start_time);
  for(size_t i = 1; i < stations.size(); i++) {
    if(profile[i] < 1e-6) {
      time_profile[i] = time_profile[i-1];
      continue;
    }
    time_profile[i] = time_profile[i-1] + (stations[i] - stations[i-1]) / profile[i];
  }
  return time_profile;
}


}