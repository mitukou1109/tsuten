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
      BACK
    };

    enum class PerformTarget
    {
      DUAL_TABLE_UPPER_F,
      DUAL_TABLE_UPPER_R,
      DUAL_TABLE_UPPER_B,
      DUAL_TABLE_UPPER_L,
      DUAL_TABLE_LOWER,
      MOVABLE_TABLE_1200,
      MOVABLE_TABLE_1500,
      MOVABLE_TABLE_1800,
      HOME
    };

    struct SensorStates
    {
      bool bumper_l;
      bool bumper_r;
    };

    static const std::vector<TableID> PERFORM_SEQUENCE;

    static const std::vector<std::array<TableID, 2>> MOVE_SKIP_TABLE_SEQUENCES;

    static const std::unordered_map<PerformPhase, uint8_t>
        PERFORM_PHASE_TO_PERFORM_FEEDBACK_PHASE;

    static const std::unordered_map<PerformPhase, std::string> PERFORM_PHASE_TEXTS;

    static const std::unordered_map<PerformTarget, TableID> PERFORM_TARGET_TO_TABLE_ID;

    static const std::unordered_map<PerformTarget, GoalID> PERFORM_TARGET_TO_GOAL_ID;

    static const std::unordered_map<PerformTarget, uint8_t>
        PERFORM_TARGET_TO_PERFORM_FEEDBACK_TARGET;

    static const tf2::Transform HOME_POSE;

    static const std::unordered_map<uint8_t, ShooterID> SHOOT_ON_TABLE_REQUEST_TO_SHOOTER_IDS;

    static const std::unordered_map<ShooterID, double> DEFAULT_SHOOTER_VALVE_ON_DURATIONS;

    void performThread();

    void launchPerformThread();

    void publishPerformFeedback(const PerformPhase &phase, const PerformTarget &target);

    void acceptPerformGoal();

    void preemptPerformAction();

    void abortPerformAction();

    void resetToInitialState();

    bool shouldSkipMovePerformPhase(const std::array<TableID, 2> &table_sequence);

    tf2::Stamped<tf2::Transform> getGoal(const GoalID &goal_id);

    void alignAtTable(const GoalID &goal_id);

    void backFromTable(const GoalID &goal_id);

    tf2::Transform getRobotBaseToGoalTF(const GoalID &goal_id);

    void initializeShooters();

    bool shootOnTable(tsuten_msgs::ShootOnTableRequest &req,
                      tsuten_msgs::ShootOnTableResponse &res);

    void resetAllShooters();

    void updateReconfigurableParameters();

    void reconfigureParameters(tsuten_behavior::BehaviorServerConfig &config, uint32_t level);

    void sensorStatesCallback(const tsuten_msgs::SensorStates &sensor_states);

    PerformTarget getPerformTargetByTableID(const TableID &table_id);

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

    double dual_table_upper_r_shooter_delay_;

    double table_approach_vel_;

    double aligning_p_gain_x_;
    double aligning_p_gain_y_;
    double aligning_p_gain_yaw_;

    boost::mutex mutex_;

    boost::condition_variable is_goal_available_cv_;

    std::unique_ptr<boost::thread> launch_perform_thread_;
    std::unique_ptr<boost::thread> perform_thread_;
  };
} // namespace tsuten_behavior