#include <tsuten_navigation/navigation_handler.hpp>

#include <geometry_msgs/Twist.h>

namespace tsuten_navigation
{
  NavigationHandler::NavigationHandler()
      : move_base_action_client_("move_base"),
        is_navigation_in_progress_(false)
  {
    ros::NodeHandle nh;

    cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  }

  bool NavigationHandler::waitForMoveBaseActionServer(const ros::Duration &timeout)
  {
    if (!move_base_action_client_.waitForServer(timeout))
    {
      ROS_ERROR("move_base action server timeout");
      return false;
    }

    return true;
  }

  NavigationHandler &NavigationHandler::startNavigation(
      const tf2::Stamped<tf2::Transform> &goal_pose)
  {
    move_base_msgs::MoveBaseGoal goal;
    tf2::toMsg(goal_pose, goal.target_pose);

    move_base_action_client_.sendGoal(goal, [this](auto &, auto &)
                                      { is_navigation_in_progress_ = false; });
    is_navigation_in_progress_ = true;

    return *this;
  }

  NavigationHandler &NavigationHandler::stopNavigation()
  {
    move_base_action_client_.cancelAllGoals();
    is_navigation_in_progress_ = false;

    return *this;
  }

  NavigationHandler &NavigationHandler::waitForNavigationToComplete()
  {
    while (is_navigation_in_progress_)
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
    }

    return *this;
  }

  NavigationHandler &NavigationHandler::commandVelocityToChassis(
      const tf2::Vector3 &cmd_vel_linear,
      const tf2::Vector3 &cmd_vel_angular)
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear = tf2::toMsg(cmd_vel_linear);
    cmd_vel.angular = tf2::toMsg(cmd_vel_angular);

    cmd_vel_pub_.publish(cmd_vel);

    return *this;
  }

  NavigationHandler &NavigationHandler::stopChassis()
  {
    return commandVelocityToChassis({0.0, 0.0, 0.0});
  }

  bool NavigationHandler::isConnectedToMoveBaseActionServer()
  {
    return move_base_action_client_.isServerConnected();
  }

  bool NavigationHandler::isNavigationInProgress()
  {
    return is_navigation_in_progress_;
  }
}