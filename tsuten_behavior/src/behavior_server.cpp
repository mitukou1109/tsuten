#include <tsuten_behavior/behavior_server.hpp>

#include <XmlRpcException.h>

#include <tsuten_msgs/ResetShooter.h>
#include <tsuten_msgs/ShootBottle.h>

namespace tsuten_behavior
{
  using TapeLEDColor = tsuten_mechanism::TapeLEDController::Color;
  using TapeLEDState = tsuten_mechanism::TapeLEDController::STATE;

  const std::unordered_map<TableID, uint8_t> TABLE_ID_TO_PERFORM_GOAL_TABLES =
      {{TableID::DUAL_TABLE_LOWER, tsuten_msgs::PerformGoal::DUAL_TABLE_LOWER},
       {TableID::DUAL_TABLE_UPPER, tsuten_msgs::PerformGoal::DUAL_TABLE_UPPER},
       {TableID::MOVABLE_TABLE_1200, tsuten_msgs::PerformGoal::MOVABLE_TABLE_1200},
       {TableID::MOVABLE_TABLE_1500, tsuten_msgs::PerformGoal::MOVABLE_TABLE_1500},
       {TableID::MOVABLE_TABLE_1800, tsuten_msgs::PerformGoal::MOVABLE_TABLE_1800}};

  const std::unordered_map<BehaviorServer::PerformPhase, std::unordered_map<TableID, uint8_t>>
      BehaviorServer::PERFORM_FEEDBACK_STATUS =
          {{PerformPhase::MOVE,
            {{TableID::DUAL_TABLE_LOWER,
              tsuten_msgs::PerformFeedback::MOVING_TO_DUAL_TABLE},
             {TableID::DUAL_TABLE_UPPER,
              tsuten_msgs::PerformFeedback::MOVING_TO_DUAL_TABLE},
             {TableID::MOVABLE_TABLE_1200,
              tsuten_msgs::PerformFeedback::MOVING_TO_MOVABLE_TABLE_1200},
             {TableID::MOVABLE_TABLE_1500,
              tsuten_msgs::PerformFeedback::MOVING_TO_MOVABLE_TABLE_1500},
             {TableID::MOVABLE_TABLE_1800,
              tsuten_msgs::PerformFeedback::MOVING_TO_MOVABLE_TABLE_1800}}},
           {PerformPhase::ALIGN,
            {{TableID::DUAL_TABLE_LOWER,
              tsuten_msgs::PerformFeedback::ALIGNING_AT_DUAL_TABLE_LOWER},
             {TableID::DUAL_TABLE_UPPER,
              tsuten_msgs::PerformFeedback::ALIGNING_AT_DUAL_TABLE_UPPER},
             {TableID::MOVABLE_TABLE_1200,
              tsuten_msgs::PerformFeedback::ALIGNING_AT_MOVABLE_TABLE_1200},
             {TableID::MOVABLE_TABLE_1500,
              tsuten_msgs::PerformFeedback::ALIGNING_AT_MOVABLE_TABLE_1500},
             {TableID::MOVABLE_TABLE_1800,
              tsuten_msgs::PerformFeedback::ALIGNING_AT_MOVABLE_TABLE_1800}}},
           {PerformPhase::SHOOT,
            {{TableID::DUAL_TABLE_LOWER,
              tsuten_msgs::PerformFeedback::SHOOTING_ON_DUAL_TABLE_LOWER},
             {TableID::DUAL_TABLE_UPPER,
              tsuten_msgs::PerformFeedback::SHOOTING_ON_DUAL_TABLE_UPPER},
             {TableID::MOVABLE_TABLE_1200,
              tsuten_msgs::PerformFeedback::SHOOTING_ON_MOVABLE_TABLE_1200},
             {TableID::MOVABLE_TABLE_1500,
              tsuten_msgs::PerformFeedback::SHOOTING_ON_MOVABLE_TABLE_1500},
             {TableID::MOVABLE_TABLE_1800,
              tsuten_msgs::PerformFeedback::SHOOTING_ON_MOVABLE_TABLE_1800}}}};

  const std::vector<TableID> BehaviorServer::PERFORM_SEQUENCE =
      {TableID::DUAL_TABLE_LOWER,
       TableID::DUAL_TABLE_UPPER,
       TableID::MOVABLE_TABLE_1200,
       TableID::MOVABLE_TABLE_1500,
       TableID::MOVABLE_TABLE_1800};

  const tf2::Transform BehaviorServer::HOME_POSE(tf2::Quaternion::getIdentity(), {0, 0, 0});

  const std::unordered_map<ShooterID, double>
      BehaviorServer::DEFAULT_SHOOTER_VALVE_ON_DURATIONS =
          {{ShooterID::DUAL_TABLE_LOWER, 0.5},
           {ShooterID::DUAL_TABLE_UPPER_L, 0.5},
           {ShooterID::DUAL_TABLE_UPPER_R, 0.5},
           {ShooterID::MOVABLE_TABLE_1200, 0.5},
           {ShooterID::MOVABLE_TABLE_1500, 0.5},
           {ShooterID::MOVABLE_TABLE_1800, 0.5}};

  BehaviorServer::BehaviorServer()
      : pnh_("~"),
        tf_listener_(tf_buffer_),
        shooter_valve_on_durations_(DEFAULT_SHOOTER_VALVE_ON_DURATIONS),
        perform_action_server_(pnh_, "perform", false),
        reconfigure_server_(reconfigure_mutex_),
        is_goal_available_(false)
  {
    pnh_.param("global_frame", global_frame_, std::string("map"));
    pnh_.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    pnh_.param("goal_distance_from_table", goal_distance_from_table_, 0.6);

    ros::NodeHandle nh;
    sensor_states_sub_ = nh.subscribe("sensor_states", 10, &BehaviorServer::sensorStatesCallback, this);

    initializeShooters();

    for (auto &shooter_name_pair : SHOOTER_NAMES)
    {
      auto &shooter_id = shooter_name_pair.first;
      auto &shooter_name = shooter_name_pair.second;

      auto shoot_bottle_service_server = pnh_.advertiseService<tsuten_msgs::ShootBottleRequest,
                                                               tsuten_msgs::ShootBottleResponse>(
          shooter_name + "/shoot_bottle",
          [this, shooter_id](auto &, auto &)
          { shooters_.at(shooter_id).shootBottle(); return true; });

      shoot_bottle_service_servers_.insert({shooter_id, shoot_bottle_service_server});
    }

    reset_shooter_service_server_ = pnh_.advertiseService<tsuten_msgs::ResetShooterRequest,
                                                          tsuten_msgs::ResetShooterResponse>(
        "reset_all_shooters",
        [this](auto &, auto &)
        { resetAllShooters(); return true; });

    shoot_on_table_service_server_ = pnh_.advertiseService(
        "shoot_on_table", &BehaviorServer::shootOnTable, this);

    perform_action_server_.registerGoalCallback(
        std::bind(&BehaviorServer::acceptPerformGoal, this));

    perform_action_server_.registerPreemptCallback(
        std::bind(&BehaviorServer::preemptPerformAction, this));

    perform_action_server_.start();

    updateReconfigurableParameters();

    reconfigure_server_.setCallback(
        boost::bind(&BehaviorServer::reconfigureParameters, this, _1, _2));

    navigation_handler_.waitForMoveBaseActionServer(ros::Duration(3.0));

    launch_perform_thread_ = std::make_unique<boost::thread>([this]
                                                             { launchPerformThread(); });
  }

  BehaviorServer::~BehaviorServer()
  {
    perform_thread_->interrupt();
    launch_perform_thread_->interrupt();
  }

  void BehaviorServer::performThread()
  {
    static const std::vector<std::array<TableID, 2>> MOVE_SKIP_TABLE_SEQUENCES =
        {{TableID::DUAL_TABLE_LOWER, TableID::DUAL_TABLE_UPPER}};

    if (!navigation_handler_.isConnectedToMoveBaseActionServer())
    {
      ROS_ERROR("move_base action server not connected");
      return;
    }

    std::vector<uint8_t> goal_tables;
    {
      boost::lock_guard<boost::mutex> lock(mutex_);
      goal_tables = perform_goal_->tables;
    }
    auto shouldShootOn = [&goal_tables](TableID table)
    { return std::any_of(goal_tables.cbegin(), goal_tables.cend(),
                         [table](const uint8_t &goal_table)
                         { return goal_table == TABLE_ID_TO_PERFORM_GOAL_TABLES.at(table); }); };

    ROS_INFO("Performance started");

    std::array<TableID, 2> table_sequence = {TableID::NONE, TableID::NONE};
    for (const auto &table : PERFORM_SEQUENCE)
    {
      if (shouldShootOn(table))
      {
        table_sequence = {table_sequence.at(1), table};
        if (std::none_of(MOVE_SKIP_TABLE_SEQUENCES.cbegin(), MOVE_SKIP_TABLE_SEQUENCES.cend(),
                         [&table_sequence](const std::array<TableID, 2> &move_skip_table_sequence)
                         { return std::is_permutation(table_sequence.cbegin(),
                                                      table_sequence.cend(),
                                                      move_skip_table_sequence.cbegin()); }))
        {
          publishPerformFeedback(PerformPhase::MOVE, table);
          tape_led_controller_.setColor(TapeLEDColor::YELLOW, TapeLEDState::STEADY);
          navigation_handler_.startNavigation(getGoal(table))
              .waitForNavigationToComplete();
        }

        publishPerformFeedback(PerformPhase::ALIGN, table);
        tape_led_controller_.setColor(TapeLEDColor::YELLOW, TapeLEDState::BLINK);
        alignAtTable(table);

        publishPerformFeedback(PerformPhase::SHOOT, table);
        tape_led_controller_.setColor(TapeLEDColor::WHITE, TapeLEDState::BLINK);
        if (table == TableID::DUAL_TABLE_UPPER)
        {
          shooters_.at(ShooterID::DUAL_TABLE_UPPER_L).shootBottle();
          boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
        }
        shooters_.at(TABLE_ID_TO_SHOOTER_ID.at(table)).shootBottle().waitUntilShootCompletes();
      }
    }

    publishPerformFeedback(PerformPhase::RETURN, TableID::NONE);
    tape_led_controller_.setColor(TapeLEDColor::YELLOW, TapeLEDState::STEADY);
    navigation_handler_.startNavigation(
                           tf2::Stamped<tf2::Transform>(HOME_POSE,
                                                        ros::Time::now(), global_frame_))
        .waitForNavigationToComplete();

    ROS_INFO("Performance completed");

    perform_action_server_.setSucceeded();
  }

  void BehaviorServer::launchPerformThread()
  {
    while (pnh_.ok())
    {
      {
        boost::unique_lock<boost::mutex> lock(mutex_);
        is_goal_available_cv_.wait(lock,
                                   [this]
                                   { return is_goal_available_; });
      }

      perform_thread_ = std::make_unique<boost::thread>([this]
                                                        { performThread(); });

      perform_thread_->join();

      resetToInitialState();
    }
  }

  void BehaviorServer::publishPerformFeedback(const PerformPhase &phase, const TableID &table_id)
  {
    tsuten_msgs::PerformFeedback feedback;
    feedback.status =
        (phase == PerformPhase::RETURN) ? tsuten_msgs::PerformFeedback::RETURNING
                                        : PERFORM_FEEDBACK_STATUS.at(phase).at(table_id);

    perform_action_server_.publishFeedback(feedback);
    ROS_INFO("Perform status: %s", PERFORM_STATUS_TEXTS.at(feedback.status).c_str());
  }

  void BehaviorServer::acceptPerformGoal()
  {
    {
      boost::lock_guard<boost::mutex> lock(mutex_);
      perform_goal_ = perform_action_server_.acceptNewGoal();
      is_goal_available_ = true;
    }

    is_goal_available_cv_.notify_all();
  }

  void BehaviorServer::preemptPerformAction()
  {
    perform_thread_->interrupt();

    perform_action_server_.setPreempted();

    ROS_INFO("Performance canceled");
  }

  void BehaviorServer::resetToInitialState()
  {
    {
      boost::lock_guard<boost::mutex> lock(mutex_);
      is_goal_available_ = false;
    }

    if (navigation_handler_.isNavigationInProgress())
    {
      navigation_handler_.stopNavigation();
    }

    navigation_handler_.stopChassis();

    resetAllShooters();

    tape_led_controller_.setColor(TapeLEDColor::RED, TapeLEDState::STEADY);
  }

  tf2::Stamped<tf2::Transform> BehaviorServer::getGoal(const TableID &table_id)
  {
    const auto &table_base_id = TABLE_ID_TO_TABLE_BASE_ID.at(table_id);

    tf2::Transform table_tf;
    while (true)
    {
      try
      {
        tf2::fromMsg(tf_buffer_.lookupTransform(
                                   global_frame_, TABLE_BASE_NAMES.at(table_base_id) + "_link",
                                   ros::Time(0),
                                   ros::Duration(1.0))
                         .transform,
                     table_tf);
        break;
      }
      catch (const tf2::TransformException &exception)
      {
        ROS_ERROR("%s", exception.what());
      }
    }

    tf2::Transform goal_transform(
        tf2::Quaternion::getIdentity(),
        {0, -(TABLE_SIZES.at(table_base_id).at(1) / 2 + goal_distance_from_table_), 0});

    return tf2::Stamped<tf2::Transform>(
        table_tf * goal_transform, ros::Time::now(), global_frame_);
  }

  void BehaviorServer::alignAtTable(const TableID &table_id)
  {
  }

  void BehaviorServer::initializeShooters()
  {
    XmlRpc::XmlRpcValue shooter_valve_on_duration_list;
    if (pnh_.getParam("shooter_valve_on_duration", shooter_valve_on_duration_list))
    {
      for (auto &shooter_valve_on_duration_pair : shooter_valve_on_duration_list)
      {
        auto &shooter_name = shooter_valve_on_duration_pair.first;
        auto &shooter_valve_on_duration = shooter_valve_on_duration_pair.second;

        if (shooter_valve_on_duration.getType() == XmlRpc::XmlRpcValue::TypeDouble)
        {
          auto shooter_name_pair_itr =
              std::find_if(
                  SHOOTER_NAMES.cbegin(), SHOOTER_NAMES.cend(),
                  [&shooter_name](const std::pair<ShooterID, std::string> &shooter_name_pair)
                  { return shooter_name == shooter_name_pair.second; });

          if (shooter_name_pair_itr != SHOOTER_NAMES.cend())
          {
            try
            {
              auto &shooter_id = (*shooter_name_pair_itr).first;
              shooter_valve_on_durations_.at(shooter_id) =
                  static_cast<double>(shooter_valve_on_duration);
            }
            catch (const XmlRpc::XmlRpcException &exception)
            {
              ROS_ERROR("Error parsing %s valve on duration: %s."
                        "Make sure to set parameter in double type.\n"
                        "Using default values.",
                        shooter_name.c_str(), exception.getMessage().c_str());
            }
          }
          else
          {
            ROS_ERROR("Invalid shooter_name: %s", shooter_name.c_str());
          }
        }
        else
        {
          ROS_ERROR("Invalid %s valve_on_duration parameter: %s",
                    shooter_name.c_str(),
                    static_cast<std::string>(shooter_valve_on_duration).c_str());
        }
      }
    }
    else
    {
      ROS_WARN("shooter_valve_on_duration not set, using default values");
    }

    for (auto &shooter_valve_on_duration_pair : shooter_valve_on_durations_)
    {
      auto &shooter_id = shooter_valve_on_duration_pair.first;
      auto &shooter_valve_on_duration = shooter_valve_on_duration_pair.second;

      shooters_.emplace(std::piecewise_construct,
                        std::forward_as_tuple(shooter_id),
                        std::forward_as_tuple(SHOOTER_NAMES.at(shooter_id),
                                              shooter_valve_on_duration));
    }
  }

  bool BehaviorServer::shootOnTable(tsuten_msgs::ShootOnTableRequest &req, tsuten_msgs::ShootOnTableResponse &res)
  {
    static ros::Timer dual_table_upper_shooter_timer_;

    switch (req.table)
    {
    case tsuten_msgs::ShootOnTableRequest::DUAL_TABLE_UPPER:
      shooters_.at(ShooterID::DUAL_TABLE_UPPER_L).shootBottle();
      dual_table_upper_shooter_timer_ = pnh_.createTimer(
          ros::Duration(1.0),
          [this](const ros::TimerEvent &)
          { shooters_.at(ShooterID::DUAL_TABLE_UPPER_R).shootBottle(); },
          true);
      break;

    case tsuten_msgs::ShootOnTableRequest::DUAL_TABLE_LOWER:
      shooters_.at(ShooterID::DUAL_TABLE_LOWER).shootBottle();
      break;

    case tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1200:
      shooters_.at(ShooterID::MOVABLE_TABLE_1200).shootBottle();
      break;

    case tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1500:
      shooters_.at(ShooterID::MOVABLE_TABLE_1500).shootBottle();
      break;

    case tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1800:
      shooters_.at(ShooterID::MOVABLE_TABLE_1800).shootBottle();
      break;

    default:
      break;
    }

    return true;
  }

  void BehaviorServer::resetAllShooters()
  {
    for (auto &shooter : shooters_)
    {
      shooter.second.resetShooter();
    }
  }

  void BehaviorServer::updateReconfigurableParameters()
  {
    BehaviorServerConfig config;
    config.goal_distance_from_table = goal_distance_from_table_;
    {
      boost::lock_guard<boost::recursive_mutex> lock(reconfigure_mutex_);
      reconfigure_server_.updateConfig(config);
    }
  }

  void BehaviorServer::reconfigureParameters(
      tsuten_behavior::BehaviorServerConfig &config, uint32_t level)
  {
    goal_distance_from_table_ = config.goal_distance_from_table;
  }

  void BehaviorServer::sensorStatesCallback(const tsuten_msgs::SensorStates &sensor_states)
  {
    sensor_states_.bumper_l = static_cast<bool>(sensor_states.bumper_l);
    sensor_states_.bumper_r = static_cast<bool>(sensor_states.bumper_r);
  }
} // namespace tsuten_behavior

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_server");

  tsuten_behavior::BehaviorServer behavior_server;

  ros::spin();

  return 0;
}