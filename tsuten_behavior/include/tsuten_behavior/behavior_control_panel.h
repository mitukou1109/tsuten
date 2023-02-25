#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <rviz/panel.h>

#include <tsuten_behavior/behavior_control_panel_ui.h>
#include <tsuten_msgs/ShootOnTable.h>
#include <tsuten_msgs/ResetShooter.h>
#include <tsuten_msgs/PerformAction.h>
#endif

namespace tsuten_behavior
{
  class BehaviorControlPanel : public rviz::Panel
  {
    Q_OBJECT

  public:
    BehaviorControlPanel(QWidget *parent = 0);

  private:
    void resetAllShooters();

    void shootOnTable(BehaviorControlPanelUI::TableID table_id);

    void startPerformance();

    void stopPerformance();

    const std::unordered_map<BehaviorControlPanelUI::TableID, uint8_t>
        TABLE_ID_TO_SHOOT_ON_TABLE_REQUEST_TABLE_ID = {
            {BehaviorControlPanelUI::TableID::DUAL_TABLE_UPPER,
             tsuten_msgs::ShootOnTableRequest::DUAL_TABLE_UPPER},
            {BehaviorControlPanelUI::TableID::DUAL_TABLE_LOWER,
             tsuten_msgs::ShootOnTableRequest::DUAL_TABLE_LOWER},
            {BehaviorControlPanelUI::TableID::MOVABLE_TABLE_1200,
             tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1200},
            {BehaviorControlPanelUI::TableID::MOVABLE_TABLE_1500,
             tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1500},
            {BehaviorControlPanelUI::TableID::MOVABLE_TABLE_1800,
             tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1800}};

    const std::unordered_map<BehaviorControlPanelUI::TableID, uint8_t>
        TABLE_ID_TO_PERFORM_GOAL_TABLE_ID = {
            {BehaviorControlPanelUI::TableID::DUAL_TABLE_UPPER,
             tsuten_msgs::PerformGoal::DUAL_TABLE_UPPER},
            {BehaviorControlPanelUI::TableID::DUAL_TABLE_LOWER,
             tsuten_msgs::PerformGoal::DUAL_TABLE_LOWER},
            {BehaviorControlPanelUI::TableID::MOVABLE_TABLE_1200,
             tsuten_msgs::PerformGoal::MOVABLE_TABLE_1200},
            {BehaviorControlPanelUI::TableID::MOVABLE_TABLE_1500,
             tsuten_msgs::PerformGoal::MOVABLE_TABLE_1500},
            {BehaviorControlPanelUI::TableID::MOVABLE_TABLE_1800,
             tsuten_msgs::PerformGoal::MOVABLE_TABLE_1800}};

    ros::NodeHandle behavior_server_nh_;

    ros::ServiceClient reset_all_shooters_service_client_;
    ros::ServiceClient shoot_on_table_service_client_;

    actionlib::SimpleActionClient<tsuten_msgs::PerformAction> perform_action_client_;

    BehaviorControlPanelUI ui_;
  };
} // namespace tsuten_behavior