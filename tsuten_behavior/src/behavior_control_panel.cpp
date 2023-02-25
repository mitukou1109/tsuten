#include <tsuten_behavior/behavior_control_panel.h>

#include <pluginlib/class_list_macros.hpp>

namespace tsuten_behavior
{
  BehaviorControlPanel::BehaviorControlPanel(QWidget *parent)
      : rviz::Panel(parent),
        behavior_server_nh_("behavior_server"),
        perform_action_client_(behavior_server_nh_, "perform"),
        ui_(parent)
  {
    setLayout(ui_.getLayout());

    reset_all_shooters_service_client_ =
        behavior_server_nh_.serviceClient<tsuten_msgs::ResetShooter>("reset_all_shooters");

    shoot_on_table_service_client_ =
        behavior_server_nh_.serviceClient<tsuten_msgs::ShootOnTable>("shoot_on_table");

    if (!perform_action_client_.waitForServer(ros::Duration(5.0)))
    {
      ROS_ERROR("perform action server timeout");
    }

    for (auto &shoot_bottle_button_pair : ui_.getShootBottleButtons())
    {
      const auto &table_id = shoot_bottle_button_pair.first;
      const auto &shoot_bottle_button = shoot_bottle_button_pair.second;

      connect(shoot_bottle_button, &QPushButton::clicked, this,
              std::bind(&BehaviorControlPanel::shootOnTable, this, table_id));
    }

    connect(ui_.getResetAllShootersButton(), &QPushButton::clicked, this, &BehaviorControlPanel::resetAllShooters);

    connect(ui_.getStartPerformanceButton(), &QPushButton::clicked, this, &BehaviorControlPanel::startPerformance);
    connect(ui_.getStopPerformanceButton(), &QPushButton::clicked, this, &BehaviorControlPanel::stopPerformance);
  }

  void BehaviorControlPanel::resetAllShooters()
  {
    tsuten_msgs::ResetShooter service;
    if (!reset_all_shooters_service_client_.call(service))
    {
      ROS_ERROR("Failed to call service reset_all_shooters");
    }
  }

  void BehaviorControlPanel::shootOnTable(BehaviorControlPanelUI::TableID table_id)
  {
    auto &table_name = ui_.TABLE_NAMES.at(table_id);
    if (ui_.confirm("shoot on table " + table_name))
    {
      tsuten_msgs::ShootOnTable service;
      service.request.table = TABLE_ID_TO_SHOOT_ON_TABLE_REQUEST_TABLE_ID.at(table_id);
      if (!shoot_on_table_service_client_.call(service))
      {
        ROS_ERROR("Failed to call service shoot_on_table (table: %s)", table_name.c_str());
      }
    }
  }

  void BehaviorControlPanel::startPerformance()
  {
    std::vector<uint8_t> tables;
    std::string table_names;

    for (auto &table_check_box_pair : ui_.getTableCheckBoxes())
    {
      const auto &table_id = table_check_box_pair.first;
      const auto &table_check_box = table_check_box_pair.second;

      if (table_check_box->isChecked())
      {
        tables.emplace_back(TABLE_ID_TO_PERFORM_GOAL_TABLE_ID.at(table_id));
        table_names += " " + ui_.TABLE_NAMES.at(table_id);
      }
    }

    if (tables.size() == 0)
    {
      ui_.warn("No tables selected");
      return;
    }

    if (ui_.confirm("perform at table(s):" + table_names))
    {
      tsuten_msgs::PerformGoal goal;
      goal.tables = tables;
      perform_action_client_.sendGoal(goal);
    }
  }

  void BehaviorControlPanel::stopPerformance()
  {
    perform_action_client_.cancelAllGoals();
  }
} // namespace tsuten_behavior

PLUGINLIB_EXPORT_CLASS(tsuten_behavior::BehaviorControlPanel, rviz::Panel);