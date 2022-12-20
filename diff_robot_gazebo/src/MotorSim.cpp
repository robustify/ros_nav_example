#include "MotorSim.hpp"

namespace diff_robot_gazebo {

  MotorSim::MotorSim(double max_speed, double max_accel, double max_decel) {
    current_speed_ = 0;
    max_speed_ = max_speed;
    max_accel_ = max_accel;
    max_decel_ = max_decel;
  }

  double MotorSim::iterate(double target_speed, double dt) {
    double target_accel = std::clamp((target_speed - current_speed_) / dt, -max_decel_, max_accel_);
    current_speed_ = std::clamp(current_speed_ + dt * target_accel, -max_speed_, max_speed_);
    return current_speed_;
  }

}
