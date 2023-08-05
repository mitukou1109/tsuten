#pragma once

#include <unordered_map>

#include <ros/ros.h>

namespace tsuten_mechanism
{
  class TapeLEDController
  {
  public:
    enum class Color
    {
      RED,
      YELLOW,
      PURPLE,
      WHITE,
      NONE
    };

    enum class STATE
    {
      STEADY,
      BLINK
    };

    TapeLEDController();

    void setColor(Color color, STATE state);

    void command(const std::array<double, 3> &rgb, STATE state);

  private:
    static const std::unordered_map<Color, std::array<double, 3>> COLOR_RGBS;

    ros::Publisher command_pub_;
  };
} // namespace tsuten_mechanism