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

  const std::unordered_map<BehaviorServer::PerformPhase, uint8_t>
      BehaviorServer::PERFORM_PHASE_TO_PERFORM_FEEDBACK_PHASES =
          {{PerformPhase::MOVE, tsuten_msgs::PerformFeedback::MOVE},
           {PerformPhase::ALIGN, tsuten_msgs::PerformFeedback::ALIGN},
           {PerformPhase::SHOOT, tsuten_msgs::PerformFeedback::SHOOT},
           {PerformPhase::BACK, tsuten_msgs::PerformFeedback::BACK}};

  const std::unordered_map<TableID, tf2::Quaternion> BehaviorServer::TABLE_POLE_TO_GOAL_QUATS =
      {{TableID::DUAL_TABLE_UPPER_F, {{0, 0, 1}, 0}},
       {TableID::DUAL_TABLE_UPPER_R, {{0, 0, 1}, M_PI_2}},
       {TableID::DUAL_TABLE_UPPER_B, {{0, 0, 1}, M_PI}},
       {TableID::DUAL_TABLE_UPPER_L, {{0, 0, 1}, -M_PI_2}},
       {TableID::DUAL_TABLE_LOWER, {{0, 0, 1}, 0}},
       {TableID::MOVABLE_TABLE_1200, {{0, 0, 1}, 0}},
       {TableID::MOVABLE_TABLE_1500, {{0, 0, 1}, 0}},
       {TableID::MOVABLE_TABLE_1800, {{0, 0, 1}, 0}}};

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
    pnh_.param("goal_distance_from_table", goal_distance_from_table_, 0.6);
    pnh_.param("aligning_p_gain_x", aligning_p_gain_x_, 1.0);
    pnh_.param("aligning_p_gain_y", aligning_p_gain_y_, 1.0);
    pnh_.param("aligning_p_gain_yaw", aligning_p_gain_yaw_, 3.0);

    ros::NodeHandle nh;
    for (const auto &table_base_name_pair : TABLE_BASE_NAMES)
    {
      auto &table_base_id = table_base_name_pair.first;
      auto &table_base_name = table_base_name_pair.second;
      table_pole_subs_.insert({table_base_id,
                               nh.subscribe<geometry_msgs::PointStamped>(
                                   table_base_name + "/table_pole",
                                   10,
                                   boost::bind(&BehaviorServer::tablePoleCallback,
                                               this,
                                               table_base_id,
                                               _1))});
    }
    sensor_states_sub_ = nh.subscribe("sensor_states", 10, &BehaviorServer::sensorStatesCallback, this);

    initializeTablePoleTFs();

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
        {{TableID::DUAL_TABLE_UPPER_F, TableID::DUAL_TABLE_LOWER},
         {TableID::DUAL_TABLE_UPPER_R, TableID::DUAL_TABLE_LOWER},
         {TableID::DUAL_TABLE_UPPER_B, TableID::DUAL_TABLE_LOWER},
         {TableID::DUAL_TABLE_UPPER_L, TableID::DUAL_TABLE_LOWER},
         {TableID::DUAL_TABLE_LOWER, TableID::DUAL_TABLE_UPPER_F}};

    static auto shouldSkipMove = [](const std::array<TableID, 2> &table_sequence)
    { return std::any_of(MOVE_SKIP_TABLE_SEQUENCES.cbegin(), MOVE_SKIP_TABLE_SEQUENCES.cend(),
                         [&table_sequence](const std::array<TableID, 2> &move_skip_table_sequence)
                         { return table_sequence == move_skip_table_sequence; }); };

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
                       { return TABLE_ID_TO_PERFORM_GOAL_TABLES.at(table_id) ==
                                perform_goal_table; }); });

    ROS_INFO("Performance started");

    for (auto table_id_itr = goal_table_ids.cbegin();
         table_id_itr != goal_table_ids.cend(); table_id_itr++)
    {
      const auto &table_id = *table_id_itr;

      if (table_id_itr != goal_table_ids.cend() &&
          !shouldSkipMove({*std::prev(table_id_itr), table_id}))
      {
        publishPerformFeedback(PerformPhase::MOVE, table_id);
        tape_led_controller_.setColor(TapeLEDColor::YELLOW, TapeLEDState::STEADY);
        navigation_handler_.startNavigation(getGoal(table_id))
            .waitForNavigationToComplete();

        publishPerformFeedback(PerformPhase::ALIGN, table_id);
        tape_led_controller_.setColor(TapeLEDColor::YELLOW, TapeLEDState::BLINK);
        alignAtTable(table_id);
      }

      publishPerformFeedback(PerformPhase::SHOOT, table_id);
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
          !shouldSkipMove({table_id, *std::next(table_id_itr)}))
      {
        publishPerformFeedback(PerformPhase::BACK, table_id);
        tape_led_controller_.setColor(TapeLEDColor::YELLOW, TapeLEDState::BLINK);
        backFromTable(table_id);
      }
    }

    publishPerformFeedback(PerformPhase::MOVE, TableID::HOME);
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
    feedback.phase = PERFORM_PHASE_TO_PERFORM_FEEDBACK_PHASES.at(phase);
    feedback.table = TABLE_ID_TO_PERFORM_FEEDBACK_TABLES.at(table_id);

    perform_action_server_.publishFeedback(feedback);
    ROS_INFO_STREAM("Perform status: " << PERFORM_FEEDBACK_PHASE_TEXTS.at(feedback.phase)
                                       << " " << TABLE_TEXTS.at(table_id));
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
    const auto &table_pole_to_goal_quat = TABLE_POLE_TO_GOAL_QUATS.at(table_id);
    tf2::Vector3 table_size(TABLE_SIZES.at(table_base_id).at(0),
                            TABLE_SIZES.at(table_base_id).at(1),
                            TABLE_SIZES.at(table_base_id).at(2));

    auto goal_tf =
        table_pole_tfs_.at(table_base_id) *
        tf2::Transform(table_pole_to_goal_quat,
                       tf2::quatRotate(table_pole_to_goal_quat, {0, 1, 0}) *
                           -(table_size + goal_distance_from_table_ * tf2::Vector3(1, 1, 1)));

    return tf2::Stamped<tf2::Transform>(goal_tf, ros::Time::now(), global_frame_);
  }

  void BehaviorServer::initializeTablePoleTFs()
  {
    for (const auto &table_base_name_pair : TABLE_BASE_NAMES)
    {
      auto &table_base_id = table_base_name_pair.first;
      auto &table_base_name = table_base_name_pair.second;

      tf2::Transform table_tf;
      while (!getTableTF(table_base_id, table_tf))
      {
        ;
      }

      table_pole_tfs_.insert({table_base_id, table_tf});
    }
  }

  void BehaviorServer::alignAtTable(const TableID &table_id)
  {
    static const tf2::Vector3 VEL_APPROACH = {0, 0.05, 0};

    while (!sensor_states_.bumper_l || !sensor_states_.bumper_r)
    {
      tf2::Transform robot_base_to_table_goal_tf;
      while (!getRobotBaseToTableGoalTF(table_id, robot_base_to_table_goal_tf))
      {
        ;
      }

      auto linear_error = robot_base_to_table_goal_tf.getOrigin();
      auto yaw_error = tf2::getYaw(robot_base_to_table_goal_tf.getRotation());

      navigation_handler_.commandVelocityToChassis(
          (linear_error * tf2::Vector3(aligning_p_gain_x_, 0, 0) + VEL_APPROACH)
              .rotate({0, 0, 1}, yaw_error),
          tf2::Vector3(0, 0, yaw_error) * tf2::Vector3(0, 0, aligning_p_gain_yaw_));
    }

    navigation_handler_.stopChassis();
  }

  void BehaviorServer::backFromTable(const TableID &table_id)
  {
    static const double LINEAR_ERROR_TOLERANCE = 0.1;
    static const double YAW_ERROR_TOLERANCE = 0.2;

    while (true)
    {
      tf2::Transform robot_base_to_table_goal_tf;
      while (!getRobotBaseToTableGoalTF(table_id, robot_base_to_table_goal_tf))
      {
        ;
      }

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

  bool BehaviorServer::getRobotBaseToTableGoalTF(const TableID &table_id,
                                                 tf2::Transform &robot_base_to_table_goal_tf)
  {
    auto table_goal_tf = getGoal(table_id);

    tf2::Transform robot_base_tf;
    try
    {
      tf2::fromMsg(
          tf_buffer_.lookupTransform(global_frame_, robot_base_frame_,
                                     ros::Time(0), ros::Duration(1.0))
              .transform,
          robot_base_tf);
    }
    catch (const tf2::TransformException &exception)
    {
      ROS_ERROR("%s", exception.what());
      return false;
    }

    robot_base_to_table_goal_tf = robot_base_tf.inverse() * table_goal_tf;

    return true;
  }

  bool BehaviorServer::getTableTF(const TableBaseID &table_id, tf2::Transform &table_tf)
  {
    try
    {
      tf2::fromMsg(
          tf_buffer_.lookupTransform(global_frame_, TABLE_BASE_NAMES.at(table_id) + "_link",
                                     ros::Time(0), ros::Duration(1.0))
              .transform,
          table_tf);

      return true;
    }
    catch (const tf2::TransformException &exception)
    {
      ROS_ERROR("%s", exception.what());

      return false;
    }
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
    config.goal_distance_from_table = goal_distance_from_table_;
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
    goal_distance_from_table_ = config.goal_distance_from_table;
    aligning_p_gain_x_ = config.aligning_p_gain_x;
    aligning_p_gain_y_ = config.aligning_p_gain_y;
    aligning_p_gain_yaw_ = config.aligning_p_gain_yaw;
  }

  void BehaviorServer::tablePoleCallback(const TableBaseID &table_base_id,
                                         const geometry_msgs::PointStamped::ConstPtr &table_pole)
  {
    tf2::fromMsg(table_pole->point, table_pole_tfs_.at(table_base_id).getOrigin());
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