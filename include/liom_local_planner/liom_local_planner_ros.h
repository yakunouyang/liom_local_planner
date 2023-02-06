//
// Created by yakunouyang on 12/5/22.
//

#ifndef SRC_LIOM_LOCAL_PLANNER_ROS_H
#define SRC_LIOM_LOCAL_PLANNER_ROS_H

// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include "liom_local_planner.h"

using namespace std;

namespace liom_local_planner {

struct LiomLocalPlannerROSConfig {
  double goal_xy_tolerance = 0.5;
  double goal_yaw_tolerance = 0.2;

  std::string odom_topic = "odom";
  std::string global_frame = "map";
};

class LiomLocalPlannerROS : public nav_core::BaseLocalPlanner {
public:

  LiomLocalPlannerROS();
  LiomLocalPlannerROS(std::string name, tf2_ros::Buffer* tf,
                      costmap_2d::Costmap2DROS* costmap_ros);

  ~LiomLocalPlannerROS();

  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  bool isGoalReached();
private:
  costmap_2d::Costmap2DROS* costmap_ros_;
  base_local_planner::OdometryHelperRos odom_helper_;
  tf2_ros::Buffer* tf_;
  bool initialized_;
  std::string name_;

  LiomLocalPlannerROSConfig config_;

  std::shared_ptr<liom_local_planner::PlannerConfig> planner_config_;
  std::shared_ptr<liom_local_planner::Environment> env_;
  std::shared_ptr<liom_local_planner::LiomLocalPlanner> planner_;
  TrajectoryPoint start_state_, goal_state_;
  ros::Publisher path_pub_;
  FullStates solution_;
  bool goal_reached_ = false;

  void ReadParameters(const ros::NodeHandle &nh);
};
};

#endif //SRC_LIOM_LOCAL_PLANNER_ROS_H
