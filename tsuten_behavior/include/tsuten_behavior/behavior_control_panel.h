#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <rviz/panel.h>

#include <tsuten_behavior/behavior_control_panel_ui.h>
#include <tsuten_behavior/constants.hpp>
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

    void shootOnTable(TableID table_id);

    void startPerformance();

    void stopPerformance();

    static const std::unordered_map<TableID, uint8_t>
        TABLE_ID_TO_SHOOT_ON_TABLE_REQUEST_TABLE_ID;

    static const std::unordered_map<TableID, uint8_t>
        TABLE_ID_TO_PERFORM_GOAL_TABLE_ID;

    ros::NodeHandle behavior_server_nh_;

    ros::ServiceClient reset_all_shooters_service_client_;
    ros::ServiceClient shoot_on_table_service_client_;

    actionlib::SimpleActionClient<tsuten_msgs::PerformAction> perform_action_client_;

    BehaviorControlPanelUI ui_;
  };
} // namespace tsuten_behavior