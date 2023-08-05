#pragma once

#include <unordered_map>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <dynamic_reconfigure/server.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <tsuten_behavior/constants.hpp>
#include <tsuten_mechanism/bottle_shooter_controller.hpp>
#include <tsuten_mechanism/tape_led_controller.hpp>
#include <tsuten_msgs/BehaviorServerConfig.h>
#include <tsuten_msgs/PerformAction.h>
#include <tsuten_msgs/SensorStates.h>
#include <tsuten_msgs/ShootOnTable.h>
#include <tsuten_navigation/navigation_handler.hpp>

namespace tsuten_behavior
{
  class BehaviorServer
  {
  public:
    BehaviorServer();

    ~BehaviorServer();

  private:
    enum class PerformPhase
    {
      MOVE,
      ALIGN,
      SHOOT,
      RETURN
    };

    struct SensorStates
    {
      bool bumper_l;
      bool bumper_r;
    };

    static const std::unordered_map<PerformPhase, std::unordered_map<TableID, uint8_t>>
        PERFORM_FEEDBACK_STATUS;

    static const std::vector<TableID> PERFORM_SEQUENCE;

    static const tf2::Transform HOME_POSE;

    static const std::unordered_map<ShooterID, double> DEFAULT_SHOOTER_VALVE_ON_DURATIONS;

    void performThread();

    void launchPerformThread();

    void publishPerformFeedback(const PerformPhase &phase, const TableID &table_id);

    void acceptPerformGoal();

    void preemptPerformAction();

    void resetToInitialState();

    tf2::Stamped<tf2::Transform> getGoal(const TableID &table_id);

    void alignAtTable(const TableID &table_id);

    void initializeShooters();

    bool shootOnTable(tsuten_msgs::ShootOnTableRequest &req, tsuten_msgs::ShootOnTableResponse &res);

    void resetAllShooters();

    void updateReconfigurableParameters();

    void reconfigureParameters(tsuten_behavior::BehaviorServerConfig &config, uint32_t level);

    void sensorStatesCallback(const tsuten_msgs::SensorStates &sensor_states);

    ros::NodeHandle pnh_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    actionlib::SimpleActionServer<tsuten_msgs::PerformAction> perform_action_server_;

    boost::recursive_mutex reconfigure_mutex_;

    dynamic_reconfigure::Server<tsuten_behavior::BehaviorServerConfig> reconfigure_server_;

    bool is_goal_available_;

    ros::Subscriber sensor_states_sub_;

    ros::ServiceServer reset_shooter_service_server_;
    ros::ServiceServer shoot_on_table_service_server_;

    std::unordered_map<ShooterID, double> shooter_valve_on_durations_;

    std::unordered_map<ShooterID, tsuten_mechanism::BottleShooterController> shooters_;

    std::unordered_map<ShooterID, ros::ServiceServer> shoot_bottle_service_servers_;

    tsuten_mechanism::TapeLEDController tape_led_controller_;

    tsuten_msgs::PerformGoalConstPtr perform_goal_;

    tsuten_navigation::NavigationHandler navigation_handler_;

    SensorStates sensor_states_;

    std::string global_frame_;
    std::string robot_base_frame_;

    double goal_distance_from_table_;

    boost::mutex mutex_;

    boost::condition_variable is_goal_available_cv_;

    std::unique_ptr<boost::thread> launch_perform_thread_;
    std::unique_ptr<boost::thread> perform_thread_;
  };
} // namespace tsuten_behavior