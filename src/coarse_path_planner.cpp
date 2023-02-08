//
// Created by yenkn on 23-1-29.
//
#include "liom_local_planner/coarse_path_planner.h"
#include "liom_local_planner/math/math_utils.h"
#include "liom_local_planner/time.h"
#include "liom_local_planner/visualization/plot.h"

#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <queue>

//#define VISUALIZE_NODE_EXPANSION
//#define VISUALIZE_GRID_MAP

namespace liom_local_planner {

constexpr int min_oneshot_freq = 2, max_oneshot_freq = 100;

bool CoarsePathPlanner::Plan(math::Pose start, math::Pose goal, std::vector<math::Pose> &result) {
  origin_ = (math::Vec2d(start) + math::Vec2d(goal)) / 2;
  is_forward_only_ = config_->vehicle.min_velocity >= 0.0;
  double forward_distance = config_->xy_resolution * M_SQRT2;
  forward_num_ = std::max(1, static_cast<int>(ceil(forward_distance / config_->step_size)));

  grid_open_pq_ = decltype(grid_open_pq_)();
  grid_open_set_.clear();
  Node2d grid_goal_node(goal, origin_, *config_);
  grid_goal_node.f_cost = 0.0;
  grid_open_pq_.emplace(grid_goal_node.index, grid_goal_node.f_cost);
  grid_open_set_.insert({grid_goal_node.index, grid_goal_node});

  open_pq_ = decltype(open_pq_)();
  open_set_.clear();
  Node3d start_node(start, origin_, *config_), goal_node(goal, origin_, *config_);
  start_node.set_cost(0, EstimateHeuristicCost(start_node));
  open_pq_.emplace(start_node.index, start_node.f_cost);
  open_set_.insert({start_node.index, start_node});

  double dist_start_to_goal = start.DistanceTo(goal);
  uint64_t oneshot_index = undefined_index;
  std::vector<math::Pose> oneshot_path;
  int walked_node_count = 0;

#ifdef VISUALIZE_NODE_EXPANSION
  std::vector<double> expand_x, expand_y;
#endif

  while(!open_pq_.empty()) {
    auto node_index = open_pq_.top().first;
    open_pq_.pop();

    auto &node = open_set_.at(node_index);
    node.is_closed = true;

#ifdef VISUALIZE_NODE_EXPANSION
    expand_x.push_back(node.pose.x());
    expand_y.push_back(node.pose.y());
#endif

    if(node_index == goal_node.index) {
      goal_node.pre_index = node.pre_index;
      break;
    }

    walked_node_count++;

    double scaled_heu_cost = (node.f_cost - node.g_cost) / dist_start_to_goal;
    int oneshot_freq = static_cast<int>(min_oneshot_freq + scaled_heu_cost * (max_oneshot_freq - min_oneshot_freq));
    bool should_check = oneshot_freq < 1 || walked_node_count % oneshot_freq == 0;

    if(should_check && CheckOneshotPath(node, goal_node, oneshot_path)) {
      oneshot_index = node.index;
      break;
    }

    int next_node_num = is_forward_only_ ? config_->next_node_num / 2 : config_->next_node_num;
    for(int i = 0; i < next_node_num; i++) {
      Node3d next_node;
      if(!ExpandNextNode(node, i, next_node)) {
        continue;
      }

      auto node_opened = open_set_.find(next_node.index);
      if(node_opened != open_set_.end() && node_opened->second.is_closed) {
        continue;
      }

      next_node.set_cost(
          node.g_cost + EvaluateExpandCost(node, next_node),
          EstimateHeuristicCost(next_node));

      if(node_opened == open_set_.end()) {
        open_set_.insert({ next_node.index, next_node });
        open_pq_.emplace( next_node.index, next_node.f_cost );
      }
      else if(next_node.g_cost < node_opened->second.g_cost) {
        node_opened->second.g_cost = next_node.g_cost;
        node_opened->second.f_cost = next_node.f_cost;
        node_opened->second.pre_index = node.index;
      }
    }
  }

  if(oneshot_index != undefined_index) { // successful oneshot
    result = TraversePath(oneshot_index);
    result.insert(result.end(), oneshot_path.begin(), oneshot_path.end());
  } else if(goal_node.pre_index != undefined_index) { // traversed to goal node (unlikely)
    result = TraversePath(goal_node.index);
  } else {
    return false;
  }

#ifdef VISUALIZE_NODE_EXPANSION
  std::vector<visualization::Color> expand_colors;
  for(int i = 0; i < expand_x.size(); i++) {
    auto color = visualization::Color::fromHSV(120 + 120.0 * i / (expand_x.size() - 1), 1.0, 1.0);
    expand_colors.push_back(color);
  }

  visualization::PlotPoints(expand_x, expand_y, expand_colors, 0.1, 1, "Expand Points");
  visualization::Trigger();
#endif

#ifdef VISUALIZE_GRID_MAP
  std::vector<double> grid_x, grid_y, grid_cost;
  for(auto &pair: grid_open_set_) {
    if (pair.second.is_closed && pair.second.f_cost < inf) {
      grid_x.push_back(origin_.x() + pair.second.x_grid * config_->grid_xy_resolution);
      grid_y.push_back(origin_.y() + pair.second.y_grid * config_->grid_xy_resolution);
      grid_cost.push_back(pair.second.f_cost);
    }
  }

  std::vector<visualization::Color> grid_colors;
  double grid_max_cost = *std::max_element(grid_cost.begin(), grid_cost.end());
  for(double cost : grid_cost) {
    grid_colors.push_back(visualization::Color::fromHSV(120.0 + 120.0 * cost / grid_max_cost, 1.0, 1.0));
  }
  visualization::PlotPoints(grid_x, grid_y, grid_colors, config_->grid_xy_resolution, 1, "Grid Map");
  visualization::Trigger();
#endif

  std::cout << "walked node: " << walked_node_count << std::endl;

  return true;
}

std::vector<math::Pose> CoarsePathPlanner::TraversePath(uint64_t node_index) {
  std::vector<math::Pose> result;
  if(open_set_.find(node_index) == open_set_.end()) {
    return result;
  }

  while(true) {
    auto &node = open_set_.at(node_index);
    uint64_t pre_index = node.pre_index;
    auto pre_node = open_set_.find(pre_index);
    if(pre_node == open_set_.end()) {
      break;
    }

    auto path = GenerateKinematicPath(pre_node->second.pose, node.is_forward, node.steering);
    for(int i = path.size() - 1; i > 0; i--) {
      result.push_back(path[i]);
    }
    node_index = pre_index;
  }

  std::reverse(result.begin(), result.end());
  return result;
}

double CoarsePathPlanner::EvaluateExpandCost(const Node3d &parent, const Node3d &node) {
  double piecewise_cost = 0.0;
  if (node.is_forward) {
    piecewise_cost += forward_num_ * config_->step_size * config_->forward_penalty;
  } else {
    piecewise_cost += forward_num_ * config_->step_size * config_->backward_penalty;
  }
  if (parent.is_forward != node.is_forward) {
    piecewise_cost += config_->gear_change_penalty;
  }
  piecewise_cost += config_->steering_penalty * std::abs(node.steering);
  piecewise_cost += config_->steering_change_penalty * std::abs(node.steering - parent.steering);
  return piecewise_cost;
}

double CoarsePathPlanner::EstimateHeuristicCost(const Node3d &node) {
  return Calculate2DCost(node);
}

constexpr int grid_directions[8][2] = {
    {-1, -1},
    {0, -1},
    {1, -1},
    {-1, 0},
    {1, 0},
    {-1, 1},
    {0, 1},
    {1, 1}
};

constexpr double grid_direction_costs[8] = {
    M_SQRT2, 1, M_SQRT2, 1, 1, M_SQRT2, 1, M_SQRT2
};

double CoarsePathPlanner::Calculate2DCost(const Node3d &node_3d) {
  Node2d node_2d(node_3d.pose, origin_, *config_);
  auto dp_node = grid_open_set_.find(node_2d.index);
  if(dp_node != grid_open_set_.end() && dp_node->second.is_closed) {
    return dp_node->second.f_cost;
  }

  if(env_->CheckBoxCollision(0.0, node_2d.GenerateBox(origin_, *config_))) {
    return inf;
  }

  while(!grid_open_pq_.empty()) {
    auto node_index = grid_open_pq_.top().first;
    grid_open_pq_.pop();

    auto &node = grid_open_set_.at(node_index);
    node.is_closed = true;

    if(node_index == node_2d.index) {
      // pause the dp expansion
      return node.f_cost;
    }

    for(int i = 0; i < 8; i++) {
      Node2d expand_node(node.x_grid + grid_directions[i][0], node.y_grid + grid_directions[i][1]);
      expand_node.pre_index = node.index;

      auto node_opened = grid_open_set_.find(expand_node.index);
      if(node_opened != grid_open_set_.end() && node_opened->second.is_closed) {
        continue;
      }

      if(env_->CheckBoxCollision(0.0, expand_node.GenerateBox(origin_, *config_))) {
        if(node_opened != grid_open_set_.end()) {
          node_opened->second.is_closed = true;
        } else {
          expand_node.is_closed = true;
          grid_open_set_.insert({ expand_node.index, expand_node });
        }
        continue;
      }

      expand_node.f_cost = node.f_cost + grid_direction_costs[i] * config_->grid_xy_resolution;

      if(node_opened == grid_open_set_.end()) {
        grid_open_set_.insert({ expand_node.index, expand_node });
        grid_open_pq_.emplace( expand_node.index, expand_node.f_cost );
      }
      else if(expand_node.f_cost < node_opened->second.f_cost) {
        node_opened->second.f_cost = expand_node.f_cost;
        node_opened->second.pre_index = node.index;
      }
    }
  }

  return inf;
}

std::vector<math::Pose> CoarsePathPlanner::GenerateKinematicPath(math::Pose pose, int is_forward, double steering) const {
  std::vector<math::Pose> path(forward_num_ + 1);
  double step_size = is_forward ? config_->step_size : -config_->step_size;

  for(int i = 0; i < forward_num_; i++) {
    path[i] = pose;
    pose.setX(pose.x() + step_size * cos(pose.theta()));
    pose.setY(pose.y() + step_size * sin(pose.theta()));
    pose.setTheta(math::NormalizeAngle(pose.theta() + step_size * tan(steering) / config_->vehicle.wheel_base));
  }

  path[forward_num_] = pose;
  return path;
}

bool CoarsePathPlanner::ExpandNextNode(const Node3d &node, int next_index, Node3d &next_node) {
  double steering;
  bool is_forward = next_index < static_cast<double>(config_->next_node_num) / 2;

  double node_res = static_cast<double>(config_->next_node_num) / 2 - 1;
  if (is_forward) {
    steering = -config_->vehicle.phi_max + 2 * config_->vehicle.phi_max / node_res * next_index;
  } else {
    int index = next_index - config_->next_node_num / 2;
    steering = -config_->vehicle.phi_max + 2 * config_->vehicle.phi_max / node_res * index;
  }

  auto path = GenerateKinematicPath(node.pose, is_forward, steering);
  for(auto &pose: path) {
    if(env_->CheckPoseCollision(0.0, pose)) {
      return false;
    }
  }

  next_node = Node3d(path.back(), origin_, *config_);
  next_node.is_forward = is_forward;
  next_node.steering = steering;
  next_node.pre_index = node.index;
  return true;
}

bool CoarsePathPlanner::CheckOneshotPath(const Node3d &node, const Node3d &goal, std::vector<math::Pose> &result) {
  if(!GenerateShortestPath(node.pose, goal.pose, result)) {
    return false;
  }

  for(auto &pose: result) {
    if(env_->CheckPoseCollision(0.0, pose)) {
      return false;
    }
  }

  return true;
}

namespace ob = ompl::base;

template<class T, class U>
std::vector<math::Pose> InterpolatePath(
    const std::shared_ptr<T> &space,
    U &path,
    const ob::ScopedState<> &start,
    const ob::ScopedState<> &goal,
    double step_size) {
  std::vector<math::Pose> result;
  ob::ScopedState<> state(space);

  bool first_time = false;
  int sample_count = std::max(1, static_cast<int>(ceil(path.length() / step_size)));

  result.resize(sample_count+1);
  for(int i = 0; i < sample_count+1; i++) {
    space->interpolate(start.get(), goal.get(), static_cast<double>(i) / sample_count, first_time, path, state.get());
    result[i].setX(state[0]);
    result[i].setY(state[1]);
    result[i].setTheta(state[2]);
  }
  return result;
}

bool CoarsePathPlanner::GenerateShortestPath(math::Pose start, math::Pose goal, std::vector<math::Pose> &result) {
  double turing_radius = config_->vehicle.wheel_base / tan(config_->vehicle.phi_max);

  std::shared_ptr<ob::SE2StateSpace> state_space;
  if(is_forward_only_) {
    state_space = std::make_shared<ob::DubinsStateSpace>(turing_radius);
  } else {
    state_space = std::make_shared<ob::ReedsSheppStateSpace>(turing_radius);
  }

  ob::ScopedState<ob::SE2StateSpace> rs_start(state_space), rs_goal(state_space);
  rs_start[0] = start.x();
  rs_start[1] = start.y();
  rs_start[2] = start.theta();
  rs_goal[0] = goal.x();
  rs_goal[1] = goal.y();
  rs_goal[2] = goal.theta();

  if(is_forward_only_) {
    auto ss = std::static_pointer_cast<ob::DubinsStateSpace>(state_space);
    auto path = ss->dubins(rs_start.get(), rs_goal.get());
    result = InterpolatePath(ss, path, rs_start, rs_goal, config_->step_size);
  } else {
    auto ss = std::static_pointer_cast<ob::ReedsSheppStateSpace>(state_space);
    auto path = ss->reedsShepp(rs_start.get(), rs_goal.get());
    result = InterpolatePath(ss, path, rs_start, rs_goal, config_->step_size);
  }

  return true;
}


}