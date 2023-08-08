#pragma once

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/utils.h>

namespace tsuten_navigation
{
  class NavigationHandler
  {
  public:
    NavigationHandler();

    bool waitForMoveBaseActionServer(const ros::Duration &timeout);

    NavigationHandler &startNavigation(const tf2::Stamped<tf2::Transform> &goal_pose);

    NavigationHandler &stopNavigation();

    NavigationHandler &waitForNavigationToComplete();

    NavigationHandler &commandVelocityToChassis(const tf2::Vector3 &cmd_vel_linear,
                                                const tf2::Vector3 &cmd_vel_angular);

    NavigationHandler &stopChassis();

    bool isConnectedToMoveBaseActionServer();

    bool isNavigationInProgress();

    bool hasNavigationSucceeded();

  private:
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_;

    bool is_navigation_in_progress_;

    ros::Publisher cmd_vel_pub_;
  };
}