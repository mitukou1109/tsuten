#pragma once

#include <unordered_map>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tsuten_behavior/constants.hpp>
#include <tsuten_mechanism/bottle_shooter_controller.hpp>
#include <tsuten_msgs/PerformAction.h>
#include <tsuten_msgs/ShootOnTable.h>

namespace tsuten_behavior
{
  class BehaviorServer
  {
  public:
    BehaviorServer();

    ~BehaviorServer();

  private:
    struct ShooterParameter
    {
      ShooterID id;
      std::string name;
      double valve_on_duration;
    };

    static const std::array<ShooterParameter, static_cast<std::size_t>(ShooterID::NUM_OF_SHOOTERS)>
        SHOOTER_PARAMETERS;

    void performThread();

    void launchPerformThread();

    void acceptPerformGoal();

    void preemptPerformAction();

    bool shootOnTable(tsuten_msgs::ShootOnTableRequest &req, tsuten_msgs::ShootOnTableResponse &res);

    void resetAllShooters();

    ros::NodeHandle pnh_;

    ros::ServiceServer reset_shooter_service_server_;
    std::unordered_map<ShooterID, ros::ServiceServer> shoot_bottle_service_servers_;
    ros::ServiceServer shoot_on_table_service_server_;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_;

    actionlib::SimpleActionServer<tsuten_msgs::PerformAction> perform_action_server_;

    std::unordered_map<ShooterID, tsuten_mechanism::BottleShooterController> shooters_;

    tsuten_msgs::PerformGoalConstPtr perform_goal_;

    bool is_goal_available_;

    boost::mutex mutex_;

    boost::condition_variable is_goal_available_cv_;

    std::unique_ptr<boost::thread> launch_perform_thread_;
    std::unique_ptr<boost::thread> perform_thread_;
  };
} // namespace tsuten_behavior