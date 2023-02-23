#pragma once

#include <ros/ros.h>

class SolenoidValveController
{
public:
  enum class State : bool
  {
    OFF = false,
    ON = true
  };

  SolenoidValveController(const std::string &valve_name);

  void setState(State state);

  State getState();

private:
  void publishState();

  ros::Publisher valve_state_pub_;

  State valve_state_;
};