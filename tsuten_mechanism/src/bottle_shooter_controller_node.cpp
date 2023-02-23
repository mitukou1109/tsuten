#include <tsuten_mechanism/bottle_shooter_controller.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "bottle_shooter_controller_node");

  ros::NodeHandle pnh("~");

  std::string bottle_shooter_name = pnh.param("bottle_shooter_name", std::string("bottle_shooter"));
  double valve_on_duration = pnh.param("valve_on_duration", 1.0);

  BottleShooterController bottle_shooter_controller(bottle_shooter_name, valve_on_duration);

  ros::spin();

  return 0;
}