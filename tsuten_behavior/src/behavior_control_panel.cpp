#include <tsuten_behavior/behavior_control_panel.h>

#include <pluginlib/class_list_macros.hpp>

#include <tsuten_msgs/ResetShooter.h>
#include <tsuten_msgs/ShootOnTable.h>

namespace tsuten_behavior
{
  const std::unordered_map<TableID, uint8_t>
      BehaviorControlPanel::TABLE_ID_TO_SHOOT_ON_TABLE_REQUEST_TABLE_ID = {
          {TableID::DUAL_TABLE_UPPER,
           tsuten_msgs::ShootOnTableRequest::DUAL_TABLE_UPPER},
          {TableID::DUAL_TABLE_LOWER,
           tsuten_msgs::ShootOnTableRequest::DUAL_TABLE_LOWER},
          {TableID::MOVABLE_TABLE_1200,
           tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1200},
          {TableID::MOVABLE_TABLE_1500,
           tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1500},
          {TableID::MOVABLE_TABLE_1800,
           tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1800}};

  const std::unordered_map<TableID, uint8_t>
      BehaviorControlPanel::TABLE_ID_TO_PERFORM_GOAL_TABLE_ID = {
          {TableID::DUAL_TABLE_UPPER,
           tsuten_msgs::PerformGoal::DUAL_TABLE_UPPER},
          {TableID::DUAL_TABLE_LOWER,
           tsuten_msgs::PerformGoal::DUAL_TABLE_LOWER},
          {TableID::MOVABLE_TABLE_1200,
           tsuten_msgs::PerformGoal::MOVABLE_TABLE_1200},
          {TableID::MOVABLE_TABLE_1500,
           tsuten_msgs::PerformGoal::MOVABLE_TABLE_1500},
          {TableID::MOVABLE_TABLE_1800,
           tsuten_msgs::PerformGoal::MOVABLE_TABLE_1800}};

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

    auto widgets = ui_.getWidgets();

    for (auto &shoot_bottle_button_pair : widgets.shoot_bottle_buttons)
    {
      const auto &table_id = shoot_bottle_button_pair.first;
      const auto &shoot_bottle_button = shoot_bottle_button_pair.second;

      connect(shoot_bottle_button, &QPushButton::clicked, this,
              std::bind(&BehaviorControlPanel::shootOnTable, this, table_id));
    }

    connect(widgets.reset_all_shooters_button, &QPushButton::clicked, this, &BehaviorControlPanel::resetAllShooters);

    connect(widgets.start_performance_button, &QPushButton::clicked, this, &BehaviorControlPanel::startPerformance);
    connect(widgets.stop_performance_button, &QPushButton::clicked, this, &BehaviorControlPanel::stopPerformance);
  }

  void BehaviorControlPanel::resetAllShooters()
  {
    tsuten_msgs::ResetShooter service;
    if (!reset_all_shooters_service_client_.call(service))
    {
      ROS_ERROR("Failed to call service reset_all_shooters");
    }
  }

  void BehaviorControlPanel::shootOnTable(TableID table_id)
  {
    auto &table_name = TABLE_NAMES.at(table_id);

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

    for (auto &table_check_box_pair : ui_.getWidgets().table_check_boxes)
    {
      const auto &table_id = table_check_box_pair.first;
      const auto &table_check_box = table_check_box_pair.second;

      if (table_check_box->isChecked())
      {
        tables.emplace_back(TABLE_ID_TO_PERFORM_GOAL_TABLE_ID.at(table_id));
        table_names += " " + TABLE_NAMES.at(table_id);
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
      perform_action_client_.sendGoal(goal,
                                      [this](const auto &, const auto &)
                                      { setUIState(UIState::IDLE); });

      setUIState(UIState::BUSY);
    }
  }

  void BehaviorControlPanel::stopPerformance()
  {
    perform_action_client_.cancelAllGoals();

    setUIState(UIState::IDLE);
  }

  void BehaviorControlPanel::setUIState(UIState state)
  {
    bool state_bool = static_cast<bool>(state);
    auto widgets = ui_.getWidgets();

    for (auto &table_check_box_pair : widgets.table_check_boxes)
    {
      table_check_box_pair.second->setCheckable(state_bool);
    }
    for (auto &shoot_bottle_button_pair : widgets.shoot_bottle_buttons)
    {
      shoot_bottle_button_pair.second->setEnabled(state_bool);
    }
    widgets.reset_all_shooters_button->setEnabled(state_bool);
    widgets.start_performance_button->setEnabled(state_bool);
    widgets.stop_performance_button->setEnabled(!state_bool);
  }
} // namespace tsuten_behavior

PLUGINLIB_EXPORT_CLASS(tsuten_behavior::BehaviorControlPanel, rviz::Panel);