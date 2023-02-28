#include <tsuten_mechanism/bottle_shooter_controller.hpp>

namespace tsuten_mechanism
{
  BottleShooterController::BottleShooterController(
      const std::string &bottle_shooter_name, const double valve_on_duration)
      : valve_controller_(bottle_shooter_name),
        is_shooting_(false)
  {
    ros::NodeHandle pnh("~");

    valve_control_timer_ =
        pnh.createTimer(ros::Duration(valve_on_duration),
                        &BottleShooterController::valveControlTimerCallback, this, false, false);
  }

  BottleShooterController &BottleShooterController::resetShooter()
  {
    valve_controller_.setState(ValveState::OFF);
    is_shooting_ = false;

    return *this;
  }

  BottleShooterController &BottleShooterController::shootBottle()
  {
    if (!is_shooting_)
    {
      valve_controller_.setState(ValveState::ON);
      is_shooting_ = true;

      valve_control_timer_.start();
    }

    return *this;
  }

  BottleShooterController &BottleShooterController::waitUntilShootCompletes(
      const ros::Duration &timeout)
  {
    ros::Time timeout_time = ros::Time::now() + timeout;

    while (is_shooting_)
    {
      ros::Duration time_left = timeout_time - ros::Time::now();
      if (timeout != ros::Duration(0) && time_left <= ros::Duration(0))
      {
        break;
      }
    }

    return *this;
  }

  void BottleShooterController::valveControlTimerCallback(const ros::TimerEvent &event)
  {
    resetShooter();
    valve_control_timer_.stop();
  }
} // namespace tsuten_mechanism