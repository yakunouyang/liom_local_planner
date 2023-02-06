//
// Created by yenkn on 23-1-20.
//
#include "math/pose.h"
#include "math/box2d.h"
#include <tuple>

#ifndef SRC_VEHICLE_MODEL_H
#define SRC_VEHICLE_MODEL_H

namespace liom_local_planner {

class VehicleModel {
public:
  /**
   * L_F, front hang length of the ego vehicle (m)
   */
  double front_hang_length = 0.96;

  /**
   * L_W, wheelbase of the ego vehicle (m)
   */
  double wheel_base = 2.80;

  /**
   * L_R, rear hang length of the ego vehicle (m)
   */
  double rear_hang_length = 0.929;

  /**
   * L_B, width of the ego vehicle (m)
   */
  double width = 1.942;

  /**
   * Upper bound of v(t) (m/s)
   */
  double max_velocity = 3.0;

  /**
   * Lower bound of v(t) (m/s)
   */
  double min_velocity = -3.0;

  /**
   * Lower and upper bounds of a(t) (m/s^2)
   */
  double max_acceleration = 1.0;

  /**
   * Upper bound of |\phi(t)| (rad)
   */
  double phi_max = 0.7;

  /**
   * Upper bound of |\omega(t)| (rad/s)
   */
  double omega_max = 0.2;

  /**
   * Number of discs covering vehicle footprint
   */
  int n_disc = 2;

  double disc_radius;
  std::vector<double> disc_coefficients;

  void InitializeDiscs() {
    double length = wheel_base + rear_hang_length + front_hang_length;
    disc_radius = 0.5 * hypot(length / n_disc, width);

    disc_coefficients.reserve(n_disc);
    for(int i = 0; i < n_disc; i++) {
      disc_coefficients.push_back(double(2 * (i + 1) - 1) / (2 * n_disc) * length - rear_hang_length);
    }
  }

  template<class T>
  std::vector<T> GetDiscPositions(const T &x, const T &y, const T &theta) const {
    std::vector<T> result; result.reserve(n_disc * 2);
    for(int i = 0; i < n_disc; i++) {
      result.push_back(x + disc_coefficients[i] * cos(theta));
      result.push_back(y + disc_coefficients[i] * sin(theta));
    }
    return result;
  }

  math::Box2d GenerateBox(const math::Pose &pose) const {
    double length = (wheel_base + rear_hang_length + front_hang_length);
    double distance = length / 2 - rear_hang_length;
    return {pose.extend(distance), pose.theta(), length, width};
  }

};
}

#endif //SRC_VEHICLE_MODEL_H
