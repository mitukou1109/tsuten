#pragma once

#include <unordered_map>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2/utils.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

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
      std::string name;
      double valve_on_duration;
    };

    static const std::unordered_map<TableID, tf2::Transform> DEFAULT_TABLE_TFS;

    static const std::unordered_map<ShooterID, ShooterParameter> SHOOTER_PARAMETERS;

    void performThread();

    void launchPerformThread();

    void acceptPerformGoal();

    void preemptPerformAction();

    void initializeTableTFs();

    void publishTF(const ros::TimerEvent &event);

    geometry_msgs::TransformStamped createTableTFMsg(TableID table_id);

    bool shootOnTable(tsuten_msgs::ShootOnTableRequest &req, tsuten_msgs::ShootOnTableResponse &res);

    void resetAllShooters();

    ros::NodeHandle pnh_;

    std::unordered_map<TableID, tf2::Transform> table_tfs_;

    actionlib::SimpleActionServer<tsuten_msgs::PerformAction> perform_action_server_;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_;

    bool is_goal_available_;

    ros::Timer publish_tf_timer_;

    ros::ServiceServer reset_shooter_service_server_;
    ros::ServiceServer shoot_on_table_service_server_;

    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    actionlib::SimpleActionServer<tsuten_msgs::PerformAction> perform_action_server_;

    std::unordered_map<ShooterID, tsuten_mechanism::BottleShooterController> shooters_;

    std::unordered_map<ShooterID, ros::ServiceServer> shoot_bottle_service_servers_;

    tsuten_msgs::PerformGoalConstPtr perform_goal_;

    std::string global_frame_;

    double tf_publish_rate_;

    boost::mutex mutex_;

    boost::condition_variable is_goal_available_cv_;

    std::unique_ptr<boost::thread> launch_perform_thread_;
    std::unique_ptr<boost::thread> perform_thread_;
  };
} // namespace tsuten_behavior