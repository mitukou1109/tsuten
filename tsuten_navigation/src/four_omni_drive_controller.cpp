#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

namespace tsuten_navigation
{
  class FourOmniDriveController
  {
  public:
    FourOmniDriveController()
    {
      ros::NodeHandle nh(""), pnh("~");

      nh.param("wheel_radius", wheel_radius_, 100.0e-3);
      pnh.param("wheel_vels_topic", wheel_vels_topic_, std::string("wheel_vels"));
      pnh.param("cmd_vel_topic", cmd_vel_topic_, std::string("cmd_vel"));

      wheel_vels_pub_ = nh.advertise<std_msgs::Float64MultiArray>(wheel_vels_topic_, 10);
      cmd_vel_sub_ = nh.subscribe(cmd_vel_topic_, 10, &FourOmniDriveController::cmdVelCallback, this);
    }

  private:
    void cmdVelCallback(const geometry_msgs::Twist &cmd_vel)
    {
      std_msgs::Float64MultiArray wheel_vels;
      wheel_vels.data.resize(4);

      for (size_t i = 0; i < 4; i++)
      {
        double theta = M_PI / 4 + i * M_PI / 2;
        wheel_vels.data.at(i) =
            (-cmd_vel.linear.x * std::sin(theta) + cmd_vel.linear.y * std::cos(theta)) /
                wheel_radius_ +
            cmd_vel.angular.z;
      }

      wheel_vels_pub_.publish(wheel_vels);
    }

    ros::Publisher wheel_vels_pub_;
    ros::Subscriber cmd_vel_sub_;

    std::string wheel_vels_topic_;
    std::string cmd_vel_topic_;

    double wheel_radius_;
  };
} // namespace tsuten_navigation

int main(int argc, char **argv)
{
  ros::init(argc, argv, "four_omni_drive_controller");

  tsuten_navigation::FourOmniDriveController four_omni_drive_controller;

  ros::spin();

  return 0;
}