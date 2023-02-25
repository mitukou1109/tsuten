#include <tsuten_mechanism/solenoid_valve_controller.hpp>

#include <std_msgs/Bool.h>

namespace tsuten_mechanism
{
  SolenoidValveController::SolenoidValveController(const std::string &valve_name)
      : valve_state_(State::OFF)
  {
    ros::NodeHandle pnh("~");

    valve_state_pub_ = pnh.advertise<std_msgs::Bool>(valve_name + "/state", 10);
  }

  void SolenoidValveController::setState(SolenoidValveController::State state)
  {
    valve_state_ = state;
    publishState();
  }

  SolenoidValveController::State SolenoidValveController::getState()
  {
    return valve_state_;
  }

  void SolenoidValveController::publishState()
  {
    std_msgs::Bool valve_state_msg;
    valve_state_msg.data = static_cast<bool>(valve_state_);
    valve_state_pub_.publish(valve_state_msg);
  }
} // namespace tsuten_mechanism