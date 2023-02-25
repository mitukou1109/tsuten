#include <tsuten_mechanism/bottle_shooter_controller.hpp>

namespace tsuten_mechanism
{
  BottleShooterController::BottleShooterController(
      const std::string &bottle_shooter_name, const double valve_on_duration)
      : valve_controller_(bottle_shooter_name)
  {
    ros::NodeHandle pnh("~");

    valve_control_timer_ =
        pnh.createTimer(ros::Duration(valve_on_duration),
                        &BottleShooterController::valveControlTimerCallback, this, false, false);
  }

  void BottleShooterController::resetShooter()
  {
    valve_controller_.setState(ValveState::OFF);
  }

  void BottleShooterController::shootBottle()
  {
    valve_controller_.setState(ValveState::ON);
    valve_control_timer_.start();
  }

  void BottleShooterController::valveControlTimerCallback(const ros::TimerEvent &event)
  {
    valve_controller_.setState(ValveState::OFF);
    valve_control_timer_.stop();
  }
} // namespace tsuten_mechanism