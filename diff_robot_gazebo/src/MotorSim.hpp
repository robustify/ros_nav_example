#pragma once
#include <ros/ros.h>

namespace diff_robot_gazebo {

  class MotorSim{
  public:
    using SharedPtr = std::shared_ptr<MotorSim>;
    MotorSim(double max_speed, double max_accel, double max_decel);

    double iterate(double target_speed, double dt);
  private:

    double current_speed_;

    double max_speed_;
    double max_accel_;
    double max_decel_;
  };

}