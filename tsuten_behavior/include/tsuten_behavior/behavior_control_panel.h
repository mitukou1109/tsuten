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
    enum class UIState : bool
    {
      BUSY = false,
      IDLE = true
    };

    void resetRobotPose();

    void simulateBumperPush();

    void shootOnTable(TableID table_id);

    void startPerformance();

    void stopPerformance();

    void setUIState(UIState state);

    static const std::unordered_map<TableID, uint8_t>
        TABLE_ID_TO_SHOOT_ON_TABLE_REQUEST_TABLE_ID;

    BehaviorControlPanelUI ui_;

    ros::NodeHandle pnh_;
    ros::NodeHandle behavior_server_nh_;

    ros::Publisher sensor_states_publisher_;

    actionlib::SimpleActionClient<tsuten_msgs::PerformAction> perform_action_client_;

    ros::ServiceClient correct_robot_pose_client_;
    ros::ServiceClient shoot_on_table_service_client_;

    std::string global_frame_;
  };
} // namespace tsuten_behavior