#include <tsuten_mechanism/solenoid_valve_controller.hpp>

#include <std_msgs/Bool.h>

namespace tsuten_mechanism
{
  SolenoidValveController::SolenoidValveController(const std::string &valve_name)
  {
    ros::NodeHandle nh;

    command_pub_ = nh.advertise<std_msgs::Bool>(valve_name + "/command", 10);
  }

  void SolenoidValveController::command(SolenoidValveController::State state)
  {
    std_msgs::Bool command_msg;
    command_msg.data = static_cast<bool>(state);
    command_pub_.publish(command_msg);
  }
} // namespace tsuten_mechanism