#pragma once

#include <ros/ros.h>

namespace tsuten_mechanism
{
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

    State valve_state_;

    ros::Publisher valve_state_pub_;
  };
} // namespace tsuten_mechanism