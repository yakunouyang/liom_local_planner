//
// Created by yenkn on 1/11/23.
//

#include "liom_local_planner/environment.h"
#include <costmap_2d/cost_values.h>

namespace liom_local_planner {

bool Environment::CheckBoxCollision(double time, const math::AABox2d &box) const {
  // TODO: reimplement using R-Tree
  for(auto &polygon: polygons_) {
    if(polygon.HasOverlap(math::Box2d(box))) {
      return true;
    }
  }

  for(auto &point: points_) {
    if(box.IsPointIn(point)) {
       return true;
    }
  }

  return false;
}

bool Environment::CheckPoseCollision(double time, math::Pose pose) const {
  auto discs = config_->vehicle.GetDiscPositions(pose.x(), pose.y(), pose.theta());
  double wh = config_->vehicle.disc_radius * 2;
  for(int i = 0; i < discs.size() / 2; i++) {
    if(CheckBoxCollision(time, math::AABox2d({discs[i * 2], discs[i * 2 + 1]}, wh, wh))) {
      return true;
    }
  }

  return false;
}

void Environment::UpdateCostmapObstacles(const costmap_2d::Costmap2D *costmap) {
  points_.clear();
  for(int i = 0; i < costmap->getSizeInCellsX()-1; i++) {
    for(int j = 0; j < costmap->getSizeInCellsY()-1; j++) {
      if(costmap->getCost(i,j) == costmap_2d::LETHAL_OBSTACLE) {
        double obs_x, obs_y;
        costmap->mapToWorld(i,j, obs_x, obs_y);
        points_.emplace_back(obs_x, obs_y);
      }
    }
  }
}

bool Environment::GenerateCorridorBox(double time, double x, double y, double radius, math::AABox2d &result) const {
  double ri = radius;
  math::AABox2d bound({-ri, -ri}, {ri, ri});

  if(CheckBoxCollision(time, bound)) {
    // initial condition not satisfied, involute to find feasible box
    int inc = 4;
    double real_x, real_y;

    do {
      int iter = inc / 4;
      uint8_t edge = inc % 4;

      real_x = x;
      real_y = y;
      if(edge == 0) {
        real_x = x - iter * 0.05;
      } else if(edge == 1) {
        real_x = x + iter * 0.05;
      } else if(edge == 2) {
        real_y = y - iter * 0.05;
      } else if(edge == 3) {
        real_y = y + iter * 0.05;
      }

      inc++;
      bound = math::AABox2d({real_x-ri, real_y-ri}, {real_x+ri, real_y+ri});
    } while(CheckBoxCollision(time, bound) && inc < config_->corridor_max_iter);
    if(inc > config_->corridor_max_iter) {
      return false;
    }

    x = real_x;
    y = real_y;
  }

  int inc = 4;
  std::bitset<4> blocked;
  double incremental[4] = {0.0};
  double step = radius * 0.2;

  do {
    int iter = inc / 4;
    uint8_t edge = inc % 4;
    inc++;

    if (blocked[edge]) continue;

    incremental[edge] = iter * step;

    math::AABox2d test({-ri - incremental[0], -ri - incremental[2]},
                 {ri + incremental[1], ri + incremental[3]});

    if (CheckBoxCollision(time, test.Offset({x, y})) || incremental[edge] >= config_->corridor_incremental_limit) {
      incremental[edge] -= step;
      blocked[edge] = true;
    }
  } while (!blocked.all() && inc < config_->corridor_max_iter);
  if (inc > config_->corridor_max_iter) {
    return false;
  }

  result = {{x - incremental[0], y - incremental[2]},
            {x + incremental[1], y + incremental[3]}};
  return true;
}


}