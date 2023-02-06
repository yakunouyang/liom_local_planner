//
// Created by yenkn on 1/10/23.
//
#include <ros/ros.h>
#include <memory>
#include "liom_local_planner/liom_local_planner.h"
#include "liom_local_planner/visualization/plot.h"

#include <nav_msgs/Path.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>

using namespace liom_local_planner;

visualization_msgs::InteractiveMarker CreateMarker(int i, const math::Polygon2d &polygon, double width, visualization::Color c) {
  visualization_msgs::InteractiveMarker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.name = "Obstacle " + std::to_string(i);
  marker.pose.orientation.w = 1.0;

  visualization_msgs::Marker polygon_marker;
  polygon_marker.header.frame_id = marker.header.frame_id;
  polygon_marker.header.stamp = ros::Time();
  polygon_marker.ns = "Obstacles";
  polygon_marker.id = i;

  polygon_marker.action = visualization_msgs::Marker::ADD;
  polygon_marker.type = visualization_msgs::Marker::LINE_STRIP;
  polygon_marker.pose.orientation.w = 1.0;
  polygon_marker.scale.x = width;
  polygon_marker.color = c.toColorRGBA();

  for (size_t i = 0; i < polygon.num_points(); i++) {
    geometry_msgs::Point pt;
    pt.x = polygon.points().at(i).x();
    pt.y = polygon.points().at(i).y();
    polygon_marker.points.push_back(pt);
  }

  geometry_msgs::Point pt;
  pt.x = polygon.points().front().x();
  pt.y = polygon.points().front().y();
  polygon_marker.points.push_back(pt);

  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back(polygon_marker);

  marker.controls.push_back(box_control);

  visualization_msgs::InteractiveMarkerControl move_control;
  move_control.name = "move_x";
  move_control.orientation.w = 0.707107f;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0.707107f;
  move_control.orientation.z = 0;
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  marker.controls.push_back(move_control);
  return marker;
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "liom_test_node");

  auto config_ = std::make_shared<PlannerConfig>();
  config_->vehicle.InitializeDiscs();

  auto env = std::make_shared<Environment>(config_);
  auto planner_ = std::make_shared<LiomLocalPlanner>(config_, env);

  ros::NodeHandle nh;
  ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("/liom_test_path", 1, false);

  interactive_markers::InteractiveMarkerServer server_("/liom_obstacle");

  std::vector<math::Polygon2d> polys = {
      math::Polygon2d({{-3, -3}, {-3, 3}, {3, 3}, {3, -3}}),
      math::Polygon2d({{-3, -3}, {-3, 3}, {3, 3}, {3, -3}}),
  };
  auto interactive_cb = [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &msg) {
    int idx = msg->marker_name.back() - '1';

    auto new_poly = polys[idx];
    new_poly.Move({ msg->pose.position.x, msg->pose.position.y });
    env->polygons().at(idx) = new_poly;
  };

  env->polygons() = polys;
  for(int i = 0; i < polys.size(); i++) {
    auto marker = CreateMarker(i+1, polys[i], 0.2, visualization::Color::Magenta);
    server_.insert(marker, interactive_cb);
  }

  server_.applyChanges();

  visualization::Init(nh, "map", "/liom_test_vis");

  TrajectoryPoint start, goal;
  FullStates solution;
  start.x = -20;
  start.y = -20;
  start.theta = 0;
  goal.x = 20;
  goal.y = 20;
  goal.theta = M_PI;
  ros::Rate r(1000);

  auto start_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, [&](const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose) {
    double x = pose->pose.pose.position.x;
    double y = pose->pose.pose.position.y;

    double min_distance = DBL_MAX;
    int idx = -1;
    for(int i = 0; i < solution.states.size(); i++) {
      double distance = hypot(solution.states[i].x - x, solution.states[i].y - y);
      if(distance < min_distance) {
        min_distance = distance;
        idx = i;
      }
    }

    start = solution.states[idx];

    ROS_INFO("intial");
  });

  while(ros::ok()) {
    if(!planner_->Plan(solution, start, goal, solution)) {
      break;
    }

    for(int i = 0; i < 1e3; i++) {
      visualization::Delete(i, "Footprints");
    }
    visualization::Trigger();

    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    for(size_t i = 0; i < solution.states.size(); i++) {
      geometry_msgs::PoseStamped pose;
      pose.header = msg.header;
      pose.pose.position.x = solution.states[i].x;
      pose.pose.position.y = solution.states[i].y;
      pose.pose.orientation = tf::createQuaternionMsgFromYaw(solution.states[i].theta);
      msg.poses.push_back(pose);

      auto box = config_->vehicle.GenerateBox(solution.states[i].pose());
      auto color = visualization::Color::White;
      color.set_alpha(0.4);
      visualization::PlotPolygon(math::Polygon2d(box), 0.05, color, i, "Footprints");
    }

    path_pub.publish(msg);
    visualization::Trigger();

    ros::spinOnce();
    r.sleep();
  }

  ros::spin();
  return 0;
}