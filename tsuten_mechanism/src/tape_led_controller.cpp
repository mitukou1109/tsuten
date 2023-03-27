#include <tsuten_mechanism/tape_led_controller.hpp>

#include <tsuten_msgs/TapeLEDCommand.h>

namespace tsuten_mechanism
{
  using TapeLEDColor = TapeLEDController::Color;

  const std::unordered_map<TapeLEDColor, std::array<double, 3>> TapeLEDController::COLOR_RGBS =
      {{TapeLEDColor::RED, {1.0, 0.0, 0.0}},
       {TapeLEDColor::YELLOW, {1.0, 1.0, 0.0}},
       {TapeLEDColor::PURPLE, {1.0, 0.0, 1.0}},
       {TapeLEDColor::WHITE, {1.0, 1.0, 1.0}},
       {TapeLEDColor::NONE, {0.0, 0.0, 0.0}}};

  TapeLEDController::TapeLEDController()
  {
    ros::NodeHandle pnh("~");

    command_pub_ = pnh.advertise<tsuten_msgs::TapeLEDCommand>("tape_led_command", 10);
  }

  void TapeLEDController::setColor(Color color, bool blink)
  {
    command(COLOR_RGBS.at(color), blink);
  }

  void TapeLEDController::command(const std::array<double, 3> &rgb, bool blink)
  {
    tsuten_msgs::TapeLEDCommand command;
    command.color.r = rgb.at(0);
    command.color.g = rgb.at(1);
    command.color.b = rgb.at(2);
    command.blink = blink;

    command_pub_.publish(command);
  }
} // namespace tsuten_mechanism