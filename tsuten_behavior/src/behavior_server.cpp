#include <tsuten_behavior/behavior_server.hpp>

#include <XmlRpcException.h>

#include <tsuten_msgs/ResetShooter.h>
#include <tsuten_msgs/ShootBottle.h>

namespace tsuten_behavior
{
  using TapeLEDColor = tsuten_mechanism::TapeLEDController::Color;
  using TapeLEDState = tsuten_mechanism::TapeLEDController::STATE;

  const std::vector<TableID> BehaviorServer::PERFORM_SEQUENCE =
      {TableID::DUAL_TABLE_UPPER_F,
       TableID::DUAL_TABLE_UPPER_R,
       TableID::DUAL_TABLE_UPPER_B,
       TableID::DUAL_TABLE_UPPER_L,
       TableID::DUAL_TABLE_LOWER,
       TableID::MOVABLE_TABLE_1200,
       TableID::MOVABLE_TABLE_1500,
       TableID::MOVABLE_TABLE_1800};

  const std::vector<std::array<TableID, 2>> BehaviorServer::MOVE_SKIP_TABLE_SEQUENCES =
      {{TableID::DUAL_TABLE_UPPER_F, TableID::DUAL_TABLE_LOWER},
       {TableID::DUAL_TABLE_UPPER_R, TableID::DUAL_TABLE_LOWER},
       {TableID::DUAL_TABLE_UPPER_B, TableID::DUAL_TABLE_LOWER},
       {TableID::DUAL_TABLE_UPPER_L, TableID::DUAL_TABLE_LOWER},
       {TableID::DUAL_TABLE_LOWER, TableID::DUAL_TABLE_UPPER_F}};

  const std::unordered_map<BehaviorServer::PerformPhase, uint8_t>
      BehaviorServer::PERFORM_PHASE_TO_PERFORM_FEEDBACK_PHASE =
          {{PerformPhase::MOVE, tsuten_msgs::PerformFeedback::MOVE},
           {PerformPhase::ALIGN, tsuten_msgs::PerformFeedback::ALIGN},
           {PerformPhase::SHOOT, tsuten_msgs::PerformFeedback::SHOOT},
           {PerformPhase::BACK, tsuten_msgs::PerformFeedback::BACK}};

  const std::unordered_map<BehaviorServer::PerformPhase, std::string>
      BehaviorServer::PERFORM_PHASE_TEXTS =
          {{PerformPhase::MOVE, "Moving to"},
           {PerformPhase::ALIGN, "Aligning at"},
           {PerformPhase::SHOOT, "Shooting on"},
           {PerformPhase::BACK, "Backing from"}};

  const std::unordered_map<BehaviorServer::PerformTarget, TableID>
      BehaviorServer::PERFORM_TARGET_TO_TABLE_ID =
          {{PerformTarget::DUAL_TABLE_UPPER_F, TableID::DUAL_TABLE_UPPER_F},
           {PerformTarget::DUAL_TABLE_UPPER_R, TableID::DUAL_TABLE_UPPER_R},
           {PerformTarget::DUAL_TABLE_UPPER_B, TableID::DUAL_TABLE_UPPER_B},
           {PerformTarget::DUAL_TABLE_UPPER_L, TableID::DUAL_TABLE_UPPER_L},
           {PerformTarget::DUAL_TABLE_LOWER, TableID::DUAL_TABLE_LOWER},
           {PerformTarget::MOVABLE_TABLE_1200, TableID::MOVABLE_TABLE_1200},
           {PerformTarget::MOVABLE_TABLE_1500, TableID::MOVABLE_TABLE_1500},
           {PerformTarget::MOVABLE_TABLE_1800, TableID::MOVABLE_TABLE_1800}};

  const std::unordered_map<BehaviorServer::PerformTarget, GoalID>
      BehaviorServer::PERFORM_TARGET_TO_GOAL_ID =
          {{PerformTarget::DUAL_TABLE_UPPER_F, GoalID::DUAL_TABLE_F},
           {PerformTarget::DUAL_TABLE_UPPER_R, GoalID::DUAL_TABLE_R},
           {PerformTarget::DUAL_TABLE_UPPER_B, GoalID::DUAL_TABLE_B},
           {PerformTarget::DUAL_TABLE_UPPER_L, GoalID::DUAL_TABLE_L},
           {PerformTarget::DUAL_TABLE_LOWER, GoalID::DUAL_TABLE_F},
           {PerformTarget::MOVABLE_TABLE_1200, GoalID::MOVABLE_TABLE_1200},
           {PerformTarget::MOVABLE_TABLE_1500, GoalID::MOVABLE_TABLE_1500},
           {PerformTarget::MOVABLE_TABLE_1800, GoalID::MOVABLE_TABLE_1800},
           {PerformTarget::HOME, GoalID::HOME}};

  const std::unordered_map<BehaviorServer::PerformTarget, uint8_t>
      BehaviorServer::PERFORM_TARGET_TO_PERFORM_FEEDBACK_TARGET =
          {{PerformTarget::DUAL_TABLE_UPPER_F, tsuten_msgs::PerformFeedback::DUAL_TABLE_UPPER_F},
           {PerformTarget::DUAL_TABLE_UPPER_R, tsuten_msgs::PerformFeedback::DUAL_TABLE_UPPER_R},
           {PerformTarget::DUAL_TABLE_UPPER_B, tsuten_msgs::PerformFeedback::DUAL_TABLE_UPPER_B},
           {PerformTarget::DUAL_TABLE_UPPER_L, tsuten_msgs::PerformFeedback::DUAL_TABLE_UPPER_L},
           {PerformTarget::DUAL_TABLE_LOWER, tsuten_msgs::PerformFeedback::DUAL_TABLE_LOWER},
           {PerformTarget::MOVABLE_TABLE_1200, tsuten_msgs::PerformFeedback::MOVABLE_TABLE_1200},
           {PerformTarget::MOVABLE_TABLE_1500, tsuten_msgs::PerformFeedback::MOVABLE_TABLE_1500},
           {PerformTarget::MOVABLE_TABLE_1800, tsuten_msgs::PerformFeedback::MOVABLE_TABLE_1800},
           {PerformTarget::HOME, tsuten_msgs::PerformFeedback::HOME}};

  const tf2::Transform BehaviorServer::HOME_POSE(tf2::Quaternion::getIdentity(), {0, 0, 0});

  const std::unordered_map<uint8_t, ShooterID>
      BehaviorServer::SHOOT_ON_TABLE_REQUEST_TO_SHOOTER_IDS =
          {{tsuten_msgs::ShootOnTableRequest::DUAL_TABLE_UPPER, ShooterID::DUAL_TABLE_UPPER_L},
           {tsuten_msgs::ShootOnTableRequest::DUAL_TABLE_LOWER, ShooterID::DUAL_TABLE_LOWER},
           {tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1200, ShooterID::MOVABLE_TABLE_1200},
           {tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1500, ShooterID::MOVABLE_TABLE_1500},
           {tsuten_msgs::ShootOnTableRequest::MOVABLE_TABLE_1800, ShooterID::MOVABLE_TABLE_1800}};

  const std::unordered_map<ShooterID, double>
      BehaviorServer::DEFAULT_SHOOTER_VALVE_ON_DURATIONS =
          {{ShooterID::DUAL_TABLE_UPPER_L, 0.5},
           {ShooterID::DUAL_TABLE_UPPER_R, 0.5},
           {ShooterID::DUAL_TABLE_LOWER, 0.5},
           {ShooterID::MOVABLE_TABLE_1200, 0.5},
           {ShooterID::MOVABLE_TABLE_1500, 0.5},
           {ShooterID::MOVABLE_TABLE_1800, 0.5}};

  BehaviorServer::BehaviorServer()
      : pnh_("~"),
        tf_listener_(tf_buffer_),
        shooter_valve_on_durations_(DEFAULT_SHOOTER_VALVE_ON_DURATIONS),
        perform_action_server_(pnh_, "perform", false),
        reconfigure_server_(reconfigure_mutex_),
        is_goal_available_(false),
        sensor_states_({false, false})
  {
    pnh_.param("global_frame", global_frame_, std::string("map"));
    pnh_.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
    pnh_.param("dual_table_upper_r_shooter_delay", dual_table_upper_r_shooter_delay_, 0.5);
    pnh_.param("table_approach_vel", table_approach_vel_, 0.1);
    pnh_.param("aligning_p_gain_x", aligning_p_gain_x_, 1.0);
    pnh_.param("aligning_p_gain_y", aligning_p_gain_y_, 1.0);
    pnh_.param("aligning_p_gain_yaw", aligning_p_gain_yaw_, 3.0);

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

    navigation_handler_.waitForMoveBaseActionServer(ros::Duration(10.0));

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
    if (!navigation_handler_.isConnectedToMoveBaseActionServer())
    {
      ROS_ERROR("move_base action server not connected");
      return;
    }

    std::vector<uint8_t> perform_goal_tables;
    {
      boost::lock_guard<boost::mutex> lock(mutex_);
      perform_goal_tables = perform_goal_->tables;
    }

    if (perform_goal_tables.empty())
    {
      ROS_ERROR("Goal has no table data");
      return;
    }

    std::vector<TableID> goal_table_ids(perform_goal_tables.size());
    std::copy_if(PERFORM_SEQUENCE.cbegin(), PERFORM_SEQUENCE.cend(), goal_table_ids.begin(),
                 [perform_goal_tables](const TableID &table_id)
                 { return std::any_of(
                       perform_goal_tables.cbegin(), perform_goal_tables.cend(),
                       [table_id](const uint8_t &perform_goal_table)
                       { return TABLE_ID_TO_PERFORM_GOAL_TABLE.at(table_id) ==
                                perform_goal_table; }); });

    ROS_INFO("Performance started");

    for (auto table_id_itr = goal_table_ids.cbegin();
         table_id_itr != goal_table_ids.cend(); table_id_itr++)
    {
      auto &table_id = *table_id_itr;
      auto goal_id = TABLE_ID_TO_GOAL_ID.at(table_id);
      const auto perform_target = getPerformTargetByTableID(table_id);

      if (table_id_itr == goal_table_ids.cbegin() ||
          !shouldSkipMovePerformPhase({*std::prev(table_id_itr), table_id}))
      {
        publishPerformFeedback(PerformPhase::MOVE, perform_target);
        tape_led_controller_.setColor(TapeLEDColor::YELLOW, TapeLEDState::STEADY);
        navigation_handler_.startNavigation(getGoal(goal_id))
            .waitForNavigationToComplete();
        if (!navigation_handler_.hasNavigationSucceeded())
        {
          abortPerformAction();
          return;
        }

        publishPerformFeedback(PerformPhase::ALIGN, perform_target);
        tape_led_controller_.setColor(TapeLEDColor::YELLOW, TapeLEDState::BLINK);
        alignAtTable(goal_id);
      }
      else
      {
        goal_id = TABLE_ID_TO_GOAL_ID.at(*std::prev(table_id_itr));
      }

      publishPerformFeedback(PerformPhase::SHOOT, perform_target);
      tape_led_controller_.setColor(TapeLEDColor::WHITE, TapeLEDState::BLINK);
      if (table_id == TableID::DUAL_TABLE_UPPER_F ||
          table_id == TableID::DUAL_TABLE_UPPER_R ||
          table_id == TableID::DUAL_TABLE_UPPER_B ||
          table_id == TableID::DUAL_TABLE_UPPER_L)
      {
        shooters_.at(ShooterID::DUAL_TABLE_UPPER_L).shootBottle();
        boost::this_thread::sleep_for(
            boost::chrono::milliseconds(std::lround(dual_table_upper_r_shooter_delay_ * 1000)));
      }
      shooters_.at(TABLE_ID_TO_SHOOTER_ID.at(table_id)).shootBottle().waitUntilShootCompletes();

      if (table_id_itr == std::prev(goal_table_ids.cend()) ||
          !shouldSkipMovePerformPhase({table_id, *std::next(table_id_itr)}))
      {
        publishPerformFeedback(PerformPhase::BACK, perform_target);
        tape_led_controller_.setColor(TapeLEDColor::YELLOW, TapeLEDState::BLINK);
        backFromTable(goal_id);
      }
    }

    publishPerformFeedback(PerformPhase::MOVE, PerformTarget::HOME);
    tape_led_controller_.setColor(TapeLEDColor::YELLOW, TapeLEDState::STEADY);
    navigation_handler_.startNavigation(
                           tf2::Stamped<tf2::Transform>(HOME_POSE,
                                                        ros::Time::now(), global_frame_))
        .waitForNavigationToComplete();
    if (!navigation_handler_.hasNavigationSucceeded())
    {
      abortPerformAction();
      return;
    }

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

  void BehaviorServer::publishPerformFeedback(
      const PerformPhase &phase, const PerformTarget &target)
  {
    tsuten_msgs::PerformFeedback feedback;
    feedback.phase = PERFORM_PHASE_TO_PERFORM_FEEDBACK_PHASE.at(phase);
    feedback.target = PERFORM_TARGET_TO_PERFORM_FEEDBACK_TARGET.at(target);

    feedback.status =
        "Perform status: " + PERFORM_PHASE_TEXTS.at(phase) + " ";
    switch (phase)
    {
    case PerformPhase::MOVE:
    case PerformPhase::ALIGN:
    case PerformPhase::BACK:
      feedback.status += GOAL_TEXTS.at(PERFORM_TARGET_TO_GOAL_ID.at(target));
      break;

    case PerformPhase::SHOOT:
      feedback.status += TABLE_TEXTS.at(PERFORM_TARGET_TO_TABLE_ID.at(target));
      break;

    default:
      break;
    }

    perform_action_server_.publishFeedback(feedback);
    ROS_INFO_STREAM(feedback.status);
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

  void BehaviorServer::abortPerformAction()
  {
    perform_action_server_.setAborted();

    ROS_INFO("Performance aborted");
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

  bool BehaviorServer::shouldSkipMovePerformPhase(const std::array<TableID, 2> &table_sequence)
  {
    return std::any_of(MOVE_SKIP_TABLE_SEQUENCES.cbegin(), MOVE_SKIP_TABLE_SEQUENCES.cend(),
                       [&table_sequence](const std::array<TableID, 2> &move_skip_table_sequence)
                       { return table_sequence == move_skip_table_sequence; });
  }

  tf2::Stamped<tf2::Transform> BehaviorServer::getGoal(const GoalID &goal_id)
  {
    tf2::Stamped<tf2::Transform> goal;
    while (true)
    {
      try
      {
        tf2::fromMsg(
            tf_buffer_.lookupTransform(global_frame_, GOAL_NAMES.at(goal_id) + "_goal",
                                       ros::Time(0), ros::Duration(0.5)),
            goal);
        break;
      }
      catch (const tf2::TransformException &exception)
      {
        ROS_ERROR("%s", exception.what());
        continue;
      }
    }

    return goal;
  }

  void BehaviorServer::alignAtTable(const GoalID &goal_id)
  {
    while (!sensor_states_.bumper_l || !sensor_states_.bumper_r)
    {
      auto robot_base_to_table_goal_tf = getRobotBaseToGoalTF(goal_id);

      auto linear_error = robot_base_to_table_goal_tf.getOrigin();
      auto yaw_error = tf2::getYaw(robot_base_to_table_goal_tf.getRotation());

      navigation_handler_.commandVelocityToChassis(
          (linear_error * tf2::Vector3(aligning_p_gain_x_, 0, 0) +
           tf2::Vector3(0, table_approach_vel_, 0))
              .rotate({0, 0, 1}, yaw_error),
          tf2::Vector3(0, 0, yaw_error) * tf2::Vector3(0, 0, aligning_p_gain_yaw_));
    }

    navigation_handler_.stopChassis();
  }

  void BehaviorServer::backFromTable(const GoalID &goal_id)
  {
    static const double LINEAR_ERROR_TOLERANCE = 0.1;
    static const double YAW_ERROR_TOLERANCE = 0.2;

    while (true)
    {
      auto robot_base_to_table_goal_tf = getRobotBaseToGoalTF(goal_id);

      auto linear_error = robot_base_to_table_goal_tf.getOrigin();
      auto yaw_error = tf2::getYaw(robot_base_to_table_goal_tf.getRotation());

      if (linear_error.length() < LINEAR_ERROR_TOLERANCE &&
          std::abs(yaw_error) < YAW_ERROR_TOLERANCE)
      {
        break;
      }
      else
      {
        navigation_handler_.commandVelocityToChassis(
            (linear_error * tf2::Vector3(aligning_p_gain_x_, aligning_p_gain_y_, 0))
                .rotate({0, 0, 1}, yaw_error),
            tf2::Vector3(0, 0, yaw_error) * tf2::Vector3(0, 0, aligning_p_gain_yaw_));
      }
    }

    navigation_handler_.stopChassis();
  }

  tf2::Transform BehaviorServer::getRobotBaseToGoalTF(const GoalID &goal_id)
  {
    bool should_wait_for_corrected_goal = true;
    tf2::Transform robot_base_to_goal_tf;

    while (true)
    {
      try
      {
        if (should_wait_for_corrected_goal &&
            tf_buffer_.canTransform(robot_base_frame_,
                                    GOAL_NAMES.at(goal_id) + "_corrected_goal",
                                    ros::Time::now(), ros::Duration(0.5)))
        {
          tf2::fromMsg(
              tf_buffer_.lookupTransform(robot_base_frame_,
                                         GOAL_NAMES.at(goal_id) + "_corrected_goal",
                                         ros::Time(0), ros::Duration(0.5))
                  .transform,
              robot_base_to_goal_tf);
        }
        else
        {
          should_wait_for_corrected_goal = false;

          tf2::fromMsg(
              tf_buffer_.lookupTransform(robot_base_frame_,
                                         GOAL_NAMES.at(goal_id) + "_goal",
                                         ros::Time(0), ros::Duration(0.5))
                  .transform,
              robot_base_to_goal_tf);
        }
        break;
      }
      catch (const tf2::TransformException &exception)
      {
        ROS_ERROR("%s", exception.what());
        continue;
      }
    }

    return robot_base_to_goal_tf;
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

    if (req.table == tsuten_msgs::ShootOnTableRequest::DUAL_TABLE_UPPER)
    {
      dual_table_upper_shooter_timer_ = pnh_.createTimer(
          ros::Duration(dual_table_upper_r_shooter_delay_),
          [this](const ros::TimerEvent &)
          { shooters_.at(ShooterID::DUAL_TABLE_UPPER_R).shootBottle(); },
          true);
    }

    shooters_.at(SHOOT_ON_TABLE_REQUEST_TO_SHOOTER_IDS.at(req.table)).shootBottle();

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
    config.table_approach_vel = table_approach_vel_;
    config.aligning_p_gain_x = aligning_p_gain_x_;
    config.aligning_p_gain_y = aligning_p_gain_y_;
    config.aligning_p_gain_yaw = aligning_p_gain_yaw_;
    {
      boost::lock_guard<boost::recursive_mutex> lock(reconfigure_mutex_);
      reconfigure_server_.updateConfig(config);
    }
  }

  void BehaviorServer::reconfigureParameters(
      tsuten_behavior::BehaviorServerConfig &config, uint32_t level)
  {
    table_approach_vel_ = config.table_approach_vel;
    aligning_p_gain_x_ = config.aligning_p_gain_x;
    aligning_p_gain_y_ = config.aligning_p_gain_y;
    aligning_p_gain_yaw_ = config.aligning_p_gain_yaw;
  }

  void BehaviorServer::sensorStatesCallback(const tsuten_msgs::SensorStates &sensor_states)
  {
    sensor_states_.bumper_l = static_cast<bool>(sensor_states.bumper_l);
    sensor_states_.bumper_r = static_cast<bool>(sensor_states.bumper_r);
  }

  BehaviorServer::PerformTarget BehaviorServer::getPerformTargetByTableID(const TableID &table_id)
  {
    return std::find_if(
               PERFORM_TARGET_TO_TABLE_ID.cbegin(), PERFORM_TARGET_TO_TABLE_ID.cend(),
               [table_id](const std::pair<PerformTarget, TableID> &perform_target_to_table_id_pair)
               { return perform_target_to_table_id_pair.second == table_id; })
        ->first;
  }
} // namespace tsuten_behavior

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_server");

  tsuten_behavior::BehaviorServer behavior_server;

  ros::spin();

  return 0;
}