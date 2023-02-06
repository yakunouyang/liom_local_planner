//
// Created by yakunouyang on 12/5/22.
//
#include "liom_local_planner/liom_local_planner_ros.h"
#include "liom_local_planner/visualization/plot.h"
#include "liom_local_planner/math/math_utils.h"

#include <base_local_planner/goal_functions.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(liom_local_planner::LiomLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace liom_local_planner {

LiomLocalPlannerROS::LiomLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

LiomLocalPlannerROS::LiomLocalPlannerROS(std::string name, tf2_ros::Buffer *tf,
                                         costmap_2d::Costmap2DROS *costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false) {
  initialize(name, tf, costmap_ros);
}

LiomLocalPlannerROS::~LiomLocalPlannerROS() {}

void LiomLocalPlannerROS::ReadParameters(const ros::NodeHandle &nh) {
  nh.param("goal_xy_tolerance", config_.goal_xy_tolerance, config_.goal_xy_tolerance);
  nh.param("goal_yaw_tolerance", config_.goal_yaw_tolerance, config_.goal_yaw_tolerance);

  nh.param("odom_topic", config_.odom_topic, config_.odom_topic);
  nh.param("global_frame", config_.global_frame, config_.global_frame);

  nh.param("coarse_xy_resolution", planner_config_->xy_resolution, planner_config_->xy_resolution);
  nh.param("coarse_theta_resolution", planner_config_->theta_resolution, planner_config_->theta_resolution);
  nh.param("coarse_step_size", planner_config_->step_size, planner_config_->step_size);
  nh.param("coarse_next_node_num", planner_config_->next_node_num, planner_config_->next_node_num);
  nh.param("coarse_grid_xy_resolution", planner_config_->grid_xy_resolution, planner_config_->grid_xy_resolution);
  nh.param("coarse_forward_penalty", planner_config_->forward_penalty, planner_config_->forward_penalty);
  nh.param("coarse_backward_penalty", planner_config_->backward_penalty, planner_config_->backward_penalty);
  nh.param("coarse_gear_change_penalty", planner_config_->gear_change_penalty, planner_config_->gear_change_penalty);
  nh.param("coarse_steering_penalty", planner_config_->steering_penalty, planner_config_->steering_penalty);
  nh.param("coarse_steering_change_penalty", planner_config_->steering_change_penalty, planner_config_->steering_change_penalty);

  nh.param("min_waypoints", planner_config_->min_nfe, planner_config_->min_nfe);
  nh.param("time_step", planner_config_->time_step, planner_config_->time_step);
  nh.param("corridor_max_iter", planner_config_->corridor_max_iter, planner_config_->corridor_max_iter);
  nh.param("corridor_incremental_limit", planner_config_->corridor_incremental_limit, planner_config_->corridor_incremental_limit);
  nh.param("weight_a", planner_config_->opti_w_a, planner_config_->opti_w_a);
  nh.param("weight_omega", planner_config_->opti_w_omega, planner_config_->opti_w_omega);
  nh.param("max_iter", planner_config_->opti_inner_iter_max, planner_config_->opti_inner_iter_max);
  nh.param("infeasible_penalty", planner_config_->opti_w_penalty0, planner_config_->opti_w_penalty0);
  nh.param("infeasible_tolerance", planner_config_->opti_varepsilon_tol, planner_config_->opti_varepsilon_tol);

  nh.param("front_hang_length", planner_config_->vehicle.front_hang_length, planner_config_->vehicle.front_hang_length);
  nh.param("wheel_base", planner_config_->vehicle.wheel_base, planner_config_->vehicle.wheel_base);
  nh.param("rear_hang_length", planner_config_->vehicle.rear_hang_length, planner_config_->vehicle.rear_hang_length);
  nh.param("width", planner_config_->vehicle.width, planner_config_->vehicle.width);

  nh.param("max_velocity", planner_config_->vehicle.max_velocity, planner_config_->vehicle.max_velocity);
  nh.param("min_velocity", planner_config_->vehicle.min_velocity, planner_config_->vehicle.min_velocity);
  nh.param("max_acceleration", planner_config_->vehicle.max_acceleration, planner_config_->vehicle.max_acceleration);
  nh.param("max_steering", planner_config_->vehicle.phi_max, planner_config_->vehicle.phi_max);
  nh.param("max_steering_rate", planner_config_->vehicle.omega_max, planner_config_->vehicle.omega_max);
  nh.param("n_disc", planner_config_->vehicle.n_disc, planner_config_->vehicle.n_disc);
}

// Take note that tf::TransformListener* has been changed to tf2_ros::Buffer* in ROS Noetic
void LiomLocalPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf,
                                     costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    name_ = name;
    initialized_ = true;

    planner_config_ = std::make_shared<liom_local_planner::PlannerConfig>();
    planner_config_->vehicle.InitializeDiscs();

    env_ = std::make_shared<liom_local_planner::Environment>(planner_config_);
    planner_ = std::make_shared<liom_local_planner::LiomLocalPlanner>(planner_config_, env_);

    ros::NodeHandle nh("~/" + name_);
    ReadParameters(nh);

    visualization::Init(nh, config_.global_frame, "liom_markers");
    odom_helper_.setOdomTopic(config_.odom_topic);

    path_pub_ = nh.advertise<nav_msgs::Path>("local_path", 1);
  }
}

bool LiomLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
  if (!initialized_) {
    ROS_ERROR("This planner has not been initialized");
    return false;
  }

  std::vector<math::Pose> path;
  path.reserve(orig_global_plan.size());
  for(auto &pose: orig_global_plan) {
    path.emplace_back(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
  }
  planner_->set_global_path(path);

  geometry_msgs::PoseStamped goal_pose;
  if(!base_local_planner::getGoalPose(*tf_, orig_global_plan, config_.global_frame, goal_pose)) {
    ROS_ERROR("get goal pose failed");
    return false;
  }

  goal_state_.x = goal_pose.pose.position.x;
  goal_state_.y = goal_pose.pose.position.y;
  goal_state_.theta = math::NormalizeAngle(tf2::getYaw(goal_pose.pose.orientation));
  goal_reached_ = false;

  return true;
}

bool LiomLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  if (!initialized_) {
    ROS_ERROR("This planner has not been initialized");
    return false;
  }

  goal_reached_ = false;
  env_->UpdateCostmapObstacles(costmap_ros_->getCostmap());

  geometry_msgs::PoseStamped current_pose;
  if(!costmap_ros_->getRobotPose(current_pose)) {
    ROS_ERROR("failed to get robot pose");
    return false;
  }
  start_state_.x = current_pose.pose.position.x;
  start_state_.y = current_pose.pose.position.y;
  start_state_.theta = math::NormalizeAngle(tf2::getYaw(current_pose.pose.orientation));

  geometry_msgs::PoseStamped current_vel;
  odom_helper_.getRobotVel(current_vel);
  start_state_.v = current_vel.pose.position.x;

  if(std::fabs(start_state_.v) > 1e-3) {
    double dtheta = tf2::getYaw(current_vel.pose.orientation);
    start_state_.phi = atan(dtheta / start_state_.v * planner_config_->vehicle.wheel_base);
  }

  if(solution_.states.size() > 1) {
    // assume control inputs reached desired values
    start_state_.a = solution_.states[1].a;
    start_state_.omega = solution_.states[1].omega;
  }

  if(std::fabs(start_state_.x - goal_state_.x) < config_.goal_xy_tolerance && std::fabs(start_state_.y - goal_state_.y) < config_.goal_xy_tolerance && std::fabs(math::AngleDiff(start_state_.theta, goal_state_.theta)) < config_.goal_yaw_tolerance) {
    goal_reached_ = true;
    solution_ = decltype(solution_)();
    return true;
  }

  bool plan_result = planner_->Plan(solution_, start_state_, goal_state_, solution_);
  if(!plan_result) {
    ROS_ERROR("local plan failed");
    return false;
  }

  nav_msgs::Path msg;
  msg.header.frame_id = config_.global_frame;
  msg.header.stamp = ros::Time::now();
  for(size_t i = 0; i < solution_.states.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = solution_.states[i].x;
    pose.pose.position.y = solution_.states[i].y;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(solution_.states[i].theta);
    msg.poses.push_back(pose);
  }

  path_pub_.publish(msg);

  if(solution_.states.size() < 2) {
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
  }

  cmd_vel.linear.x = solution_.states[1].v;
  cmd_vel.angular.z = solution_.states[1].phi;
  return true;
}

bool LiomLocalPlannerROS::isGoalReached() {
  if (!initialized_) {
    ROS_ERROR("This planner has not been initialized");
    return false;
  }

  return goal_reached_;
}
}
