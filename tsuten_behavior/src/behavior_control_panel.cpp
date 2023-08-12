#include <tsuten_behavior/behavior_control_panel.h>

#include <pluginlib/class_list_macros.hpp>

#include <tsuten_msgs/CorrectRobotPose.h>
#include <tsuten_msgs/SensorStates.h>
#include <tsuten_msgs/ShootOnTable.h>

namespace tsuten_behavior
{
  const std::unordered_map<TableID, uint8_t>
      BehaviorControlPanel::TABLE_ID_TO_SHOOT_ON_TABLE_REQUEST_TABLE_ID =
          {{TableID::DUAL_TABLE_UPPER,
            tsuten_msgs::ShootOnTableRequest::DUAL_TABLE_UPPER},
           {TableID::DUAL_TABLE_LOWER,
            tsuten_msgs::ShootOnTableRequest::DUAL_TABLE_LOWER},
           {TableID::MOVABLE_TABLE_1200,
            tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1200},
           {TableID::MOVABLE_TABLE_1500,
            tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1500},
           {TableID::MOVABLE_TABLE_1800,
            tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1800}};

  BehaviorControlPanel::BehaviorControlPanel(QWidget *parent)
      : rviz::Panel(parent),
        ui_(parent),
        pnh_("~"),
        behavior_server_nh_("behavior_server"),
        perform_action_client_(behavior_server_nh_, "perform")
  {
    setLayout(ui_.getLayout());

    pnh_.param("global_frame", global_frame_, std::string("map"));

    ros::NodeHandle nh;
    sensor_states_publisher_ = nh.advertise<tsuten_msgs::SensorStates>("sensor_states", 1);

    ros::NodeHandle localization_helper_nh("localization_helper");
    correct_robot_pose_client_ =
        localization_helper_nh.serviceClient<tsuten_msgs::CorrectRobotPose>("correct_robot_pose");

    shoot_on_table_service_client_ =
        behavior_server_nh_.serviceClient<tsuten_msgs::ShootOnTable>("shoot_on_table");

    if (!perform_action_client_.waitForServer(ros::Duration(5.0)))
    {
      ROS_ERROR("perform action server timeout");
    }

    const auto &widgets = ui_.getWidgets();

    connect(widgets.table_check_boxes.at(TableID::DUAL_TABLE_UPPER),
            &QCheckBox::stateChanged, this,
            [widgets]()
            { widgets.dual_table_upper_combo_box_->setEnabled(
                  widgets.table_check_boxes.at(TableID::DUAL_TABLE_UPPER)->isChecked()); });

    connect(widgets.simulate_bumper_push_button, &QPushButton::clicked, this,
            &BehaviorControlPanel::simulateBumperPush);

    for (auto &shoot_bottle_button_pair : widgets.shoot_bottle_buttons)
    {
      const auto &table_id = shoot_bottle_button_pair.first;
      const auto &shoot_bottle_button = shoot_bottle_button_pair.second;

      connect(shoot_bottle_button, &QPushButton::clicked, this,
              std::bind(&BehaviorControlPanel::shootOnTable, this, table_id));
    }

    connect(widgets.reset_robot_pose_button, &QPushButton::clicked, this,
            &BehaviorControlPanel::resetRobotPose);

    connect(widgets.start_performance_button, &QPushButton::clicked, this,
            &BehaviorControlPanel::startPerformance);
    connect(widgets.stop_performance_button, &QPushButton::clicked, this,
            &BehaviorControlPanel::stopPerformance);
  }

  void BehaviorControlPanel::simulateBumperPush()
  {
    static const double BUMPER_PUSH_DURATION = 0.5;
    static ros::Timer bumper_release_timer;

    bumper_release_timer = pnh_.createTimer(
        ros::Duration(BUMPER_PUSH_DURATION),
        [this](const auto &)
        {
          tsuten_msgs::SensorStates sensor_states;
          sensor_states.bumper_l = false;
          sensor_states.bumper_r = false;
          sensor_states_publisher_.publish(sensor_states);
        },
        true);

    tsuten_msgs::SensorStates sensor_states;
    sensor_states.bumper_l = true;
    sensor_states.bumper_r = true;
    sensor_states_publisher_.publish(sensor_states);
  }

  void BehaviorControlPanel::resetRobotPose()
  {
    geometry_msgs::PoseStamped origin_pose;
    origin_pose.header.frame_id = global_frame_;
    origin_pose.header.stamp = ros::Time::now();
    origin_pose.pose.orientation.w = 1.0;

    tsuten_msgs::CorrectRobotPose service;
    service.request.robot_pose = origin_pose;
    if (!correct_robot_pose_client_.call(service))
    {
      ROS_ERROR("Failed to call service correct_robot_pose");
    }
  }

  void BehaviorControlPanel::shootOnTable(TableID table_id)
  {
    auto &table_text = TABLE_TEXTS.at(table_id);

    if (ui_.confirm("shoot on " + table_text))
    {
      tsuten_msgs::ShootOnTable service;
      service.request.table = TABLE_ID_TO_SHOOT_ON_TABLE_REQUEST_TABLE_ID.at(table_id);
      if (!shoot_on_table_service_client_.call(service))
      {
        ROS_ERROR("Failed to call service shoot_on_table (table: %s)", table_text.c_str());
      }
    }
  }

  void BehaviorControlPanel::startPerformance()
  {
    std::vector<uint8_t> tables;
    std::string tables_text;

    for (auto &table_check_box_pair : ui_.getWidgets().table_check_boxes)
    {
      auto table_id = table_check_box_pair.first;
      const auto &table_check_box = table_check_box_pair.second;

      if (table_check_box->isChecked())
      {
        if (table_id == TableID::DUAL_TABLE_UPPER)
        {
          table_id = (*std::next(BehaviorControlPanelUI::DUAL_TABLE_UPPER_COMBO_BOX_ITEMS.cbegin(),
                                 ui_.getWidgets().dual_table_upper_combo_box_->currentIndex()))
                         .first;
        }
        tables.emplace_back(TABLE_ID_TO_PERFORM_GOAL_TABLE.at(table_id));
        tables_text += " " + TABLE_TEXTS.at(table_id) + ",";
      }
    }

    if (tables.size() == 0)
    {
      ui_.warn("No tables selected");
      return;
    }
    else
    {
      tables_text.pop_back();
    }

    if (ui_.confirm("perform at:" + tables_text))
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
      table_check_box_pair.second->setEnabled(state_bool);
    }
    widgets.dual_table_upper_combo_box_->setEnabled(state_bool);
    for (auto &shoot_bottle_button_pair : widgets.shoot_bottle_buttons)
    {
      shoot_bottle_button_pair.second->setEnabled(state_bool);
    }
    widgets.reset_robot_pose_button->setEnabled(state_bool);
    widgets.start_performance_button->setEnabled(state_bool);
    widgets.stop_performance_button->setEnabled(!state_bool);
  }
} // namespace tsuten_behavior

PLUGINLIB_EXPORT_CLASS(tsuten_behavior::BehaviorControlPanel, rviz::Panel);