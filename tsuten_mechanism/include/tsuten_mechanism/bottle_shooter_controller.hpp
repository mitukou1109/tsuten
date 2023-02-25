#pragma once

#include <ros/ros.h>

#include <tsuten_mechanism/solenoid_valve_controller.hpp>

namespace tsuten_mechanism
{
  class BottleShooterController
  {
  public:
    BottleShooterController(const std::string &bottle_shooter_name, const double valve_on_duration);

    void resetShooter();

    void shootBottle();

  private:
    using ValveState = SolenoidValveController::State;

    void valveControlTimerCallback(const ros::TimerEvent &event);

    ros::Timer valve_control_timer_;

    SolenoidValveController valve_controller_;
  };
} // namespace tsuten_mechanism