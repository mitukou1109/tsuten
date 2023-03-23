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

    void command(State state);

  private:
    ros::Publisher command_pub_;
  };
} // namespace tsuten_mechanism