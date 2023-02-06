//
// Created by yenkn on 1/11/23.
//

#ifndef SRC_ENVIRONMENT_H
#define SRC_ENVIRONMENT_H
#include <vector>
#include <costmap_2d/costmap_2d.h>

#include "math/polygon2d.h"
#include "planner_config.h"

namespace liom_local_planner {

class Environment {
public:
  explicit Environment(std::shared_ptr<PlannerConfig> config): config_(std::move(config)) {}

  std::vector<math::Polygon2d> &polygons() { return polygons_; }

  bool CheckPoseCollision(double time, math::Pose pose) const;

  bool GenerateCorridorBox(double time, double x, double y, double radius, math::AABox2d &result) const;

  bool CheckBoxCollision(double time, const math::AABox2d &box) const;

  void UpdateCostmapObstacles(const costmap_2d::Costmap2D *costmap);

private:
  std::shared_ptr<PlannerConfig> config_;
  std::vector<math::Polygon2d> polygons_;
  std::vector<math::Vec2d> points_;
};

}

#endif //SRC_ENVIRONMENT_H
