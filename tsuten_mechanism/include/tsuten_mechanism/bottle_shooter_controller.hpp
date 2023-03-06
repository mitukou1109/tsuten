#pragma once

#include <ros/ros.h>

#include <tsuten_mechanism/solenoid_valve_controller.hpp>

namespace tsuten_mechanism
{
  class BottleShooterController
  {
  public:
    BottleShooterController(const std::string &bottle_shooter_name,
                            const double valve_on_duration);

    BottleShooterController &resetShooter();

    BottleShooterController &shootBottle();

    BottleShooterController &waitUntilShootCompletes();

  private:
    using ValveState = SolenoidValveController::State;

    void valveControlTimerCallback(const ros::TimerEvent &event);

    SolenoidValveController valve_controller_;

    bool is_shooting_;

    ros::Timer valve_control_timer_;
  };
} // namespace tsuten_mechanism