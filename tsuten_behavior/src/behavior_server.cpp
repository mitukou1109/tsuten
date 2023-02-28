#include <tsuten_behavior/behavior_server.hpp>

#include <XmlRpcException.h>

#include <tsuten_msgs/ResetShooter.h>
#include <tsuten_msgs/ShootBottle.h>

namespace tsuten_behavior
{
  const std::unordered_map<TableID, tf2::Transform> BehaviorServer::DEFAULT_TABLE_TFS =
      {{TableID::DUAL_TABLE, tf2::Transform({{0, 0, 1}, M_PI / 2}, {2.5, 2.4, 0})},
       {TableID::MOVABLE_TABLE_1200, tf2::Transform({{0, 0, 1}, M_PI / 2}, {4.5, 1.9, 0})},
       {TableID::MOVABLE_TABLE_1500, tf2::Transform({{0, 0, 1}, M_PI / 2}, {4.5, 1.9, 0})},
       {TableID::MOVABLE_TABLE_1800, tf2::Transform({{0, 0, 1}, M_PI / 2}, {6.5, 1.9, 0})}};

  const std::unordered_map<ShooterID, double>
      BehaviorServer::DEFAULT_SHOOTER_VALVE_ON_DURATIONS_ =
          {{ShooterID::DUAL_TABLE_LOWER, 0.5},
           {ShooterID::DUAL_TABLE_UPPER_L, 0.5},
           {ShooterID::DUAL_TABLE_UPPER_R, 0.5},
           {ShooterID::MOVABLE_TABLE_1200, 0.5},
           {ShooterID::MOVABLE_TABLE_1500, 0.5},
           {ShooterID::MOVABLE_TABLE_1800, 0.5}};

  BehaviorServer::BehaviorServer()
      : pnh_("~"),
        table_tfs_(DEFAULT_TABLE_TFS),
        shooter_valve_on_durations_(DEFAULT_SHOOTER_VALVE_ON_DURATIONS_),
        perform_action_server_(pnh_, "perform", false),
        move_base_action_client_("move_base"),
        is_goal_available_(false)
  {
    initializeTableTFs();

    for (auto &table_tf_pair : table_tfs_)
    {
      auto &table_id = table_tf_pair.first;
      auto &table_position = table_tf_pair.second.getOrigin();

      pnh_.setParam(TABLE_NAMES.at(table_id) + "_position_y", table_position.y());
    }

    pnh_.param("global_frame", global_frame_, std::string("map"));
    pnh_.param("tf_publish_rate", tf_publish_rate_, 10.0);

    static_tf_broadcaster_.sendTransform(createTableTFMsg(TableID::DUAL_TABLE));

    publish_tf_timer_ = pnh_.createTimer(ros::Rate(tf_publish_rate_), &BehaviorServer::publishTF, this);

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

    if (!move_base_action_client_.waitForServer(ros::Duration(3.0)))
    {
      ROS_ERROR("move_base action server timeout");
    }

    reconfigure_server_.setCallback(boost::bind(&BehaviorServer::reconfigureParameters, this, _1, _2));

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
    std::vector<uint8_t> tables;
    auto isDirectedToPerformAt = [&tables](uint8_t table)
    { return std::find(tables.cbegin(), tables.cend(), table) != tables.cend(); };

    {
      boost::lock_guard<boost::mutex> lock(mutex_);
      tables = perform_goal_->tables;
    }

    ROS_INFO("Performance started");

    boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));

    if (isDirectedToPerformAt(tsuten_msgs::PerformGoal::DUAL_TABLE_UPPER))
    {
      publishPerformFeedback(tsuten_msgs::PerformFeedback::SHOOTING_ON_DUAL_TABLE_UPPER);
      shooters_.at(ShooterID::DUAL_TABLE_UPPER_L).shootBottle();
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
      shooters_.at(ShooterID::DUAL_TABLE_UPPER_R).shootBottle().waitUntilShootCompletes();
    }

    if (isDirectedToPerformAt(tsuten_msgs::PerformGoal::DUAL_TABLE_LOWER))
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(3000));
      publishPerformFeedback(tsuten_msgs::PerformFeedback::SHOOTING_ON_DUAL_TABLE_LOWER);
      shooters_.at(ShooterID::DUAL_TABLE_LOWER).shootBottle().waitUntilShootCompletes();
    }

    if (isDirectedToPerformAt(tsuten_msgs::PerformGoal::MOVABLE_TABLE_1200))
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(3000));
      publishPerformFeedback(tsuten_msgs::PerformFeedback::SHOOTING_ON_MOVABLE_TABLE_1200);
      shooters_.at(ShooterID::MOVABLE_TABLE_1200).shootBottle().waitUntilShootCompletes();
    }

    if (isDirectedToPerformAt(tsuten_msgs::PerformGoal::MOVABLE_TABLE_1500))
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(3000));
      publishPerformFeedback(tsuten_msgs::PerformFeedback::SHOOTING_ON_MOVABLE_TABLE_1500);
      shooters_.at(ShooterID::MOVABLE_TABLE_1500).shootBottle().waitUntilShootCompletes();
    }

    if (isDirectedToPerformAt(tsuten_msgs::PerformGoal::MOVABLE_TABLE_1800))
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(3000));
      publishPerformFeedback(tsuten_msgs::PerformFeedback::SHOOTING_ON_MOVABLE_TABLE_1800);
      shooters_.at(ShooterID::MOVABLE_TABLE_1800).shootBottle().waitUntilShootCompletes();
    }

    ROS_INFO("Performance completed");
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

      perform_action_server_.setSucceeded();

      is_goal_available_ = false;
    }
  }

  void BehaviorServer::publishPerformFeedback(uint8_t status)
  {
    tsuten_msgs::PerformFeedback feedback;
    feedback.status = status;

    perform_action_server_.publishFeedback(feedback);
    ROS_INFO("Perform status: %s", PERFORM_STATUS_TEXTS.at(status).c_str());
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
    {
      boost::lock_guard<boost::mutex> lock(mutex_);
      is_goal_available_ = false;
    }

    perform_thread_->interrupt();

    perform_action_server_.setPreempted();
    ROS_INFO("Performance canceled");

    resetAllShooters();
  }

  void BehaviorServer::initializeTableTFs()
  {
    XmlRpc::XmlRpcValue table_position_list;
    if (pnh_.getParam("table_position", table_position_list))
    {
      for (auto &table_position_pair : table_position_list)
      {
        auto &table_name = table_position_pair.first;
        auto &table_position = table_position_pair.second;

        if (table_position.getType() == XmlRpc::XmlRpcValue::TypeArray)
        {
          auto table_name_pair_itr =
              std::find_if(TABLE_NAMES.cbegin(), TABLE_NAMES.cend(),
                           [&table_name](const std::pair<TableID, std::string> &table_name_pair)
                           { return table_name == table_name_pair.second &&
                                    table_name != TABLE_NAMES.at(TableID::DUAL_TABLE_LOWER) &&
                                    table_name != TABLE_NAMES.at(TableID::DUAL_TABLE_UPPER); });

          if (table_name_pair_itr != TABLE_NAMES.cend())
          {
            auto &table_id = (*table_name_pair_itr).first;
            try
            {
              table_tfs_.at(table_id) =
                  tf2::Transform(DEFAULT_TABLE_TFS.at(table_id).getRotation(),
                                 tf2::Vector3(static_cast<double>(table_position[0]),
                                              static_cast<double>(table_position[1]),
                                              static_cast<double>(table_position[2])));
            }
            catch (const XmlRpc::XmlRpcException &exception)
            {
              ROS_ERROR(
                  "Error parsing %s position: %s. Make sure to set parameter in double type.\n"
                  "Using default values.",
                  table_name.c_str(), exception.getMessage().c_str());
            }
          }
          else
          {
            ROS_ERROR("Invalid table_name: %s", table_name.c_str());
          }
        }
        else
        {
          ROS_ERROR("Invalid %s position parameter: %s",
                    table_name.c_str(),
                    static_cast<std::string>(table_position).c_str());
        }
      }
    }
    else
    {
      ROS_WARN("Table positions not set, using default values");
    }

    for (auto &table_tf_pair : table_tfs_)
    {
      auto &table_id = table_tf_pair.first;
      auto &table_tf = table_tf_pair.second;

      table_tf.getOrigin().setZ(TABLE_SIZES.at(table_id).at(2) / 2); // for visualization
    }
  }

  void BehaviorServer::publishTF(const ros::TimerEvent &event)
  {
    for (auto &table_tf_pair : table_tfs_)
    {
      auto &table_id = table_tf_pair.first;

      if (table_id == TableID::DUAL_TABLE)
      {
        continue;
      }

      tf_broadcaster_.sendTransform(createTableTFMsg(table_id));
    }
  }

  geometry_msgs::TransformStamped BehaviorServer::createTableTFMsg(TableID table_id)
  {
    geometry_msgs::TransformStamped table_tf_msg;
    table_tf_msg.header.frame_id = global_frame_;
    table_tf_msg.header.stamp = ros::Time::now();
    table_tf_msg.child_frame_id = TABLE_NAMES.at(table_id) + "_link";
    table_tf_msg.transform = tf2::toMsg(table_tfs_.at(table_id));

    return table_tf_msg;
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

  void BehaviorServer::reconfigureParameters(
      tsuten_behavior::BehaviorServerConfig &config, uint32_t level)
  {
    table_tfs_.at(TableID::MOVABLE_TABLE_1200)
        .getOrigin()
        .setY(config.movable_table_1200_position_y);
    table_tfs_.at(TableID::MOVABLE_TABLE_1500)
        .getOrigin()
        .setY(config.movable_table_1500_position_y);
    table_tfs_.at(TableID::MOVABLE_TABLE_1800)
        .getOrigin()
        .setY(config.movable_table_1800_position_y);
  }
} // namespace tsuten_behavior

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_server");

  tsuten_behavior::BehaviorServer behavior_server;

  ros::spin();

  return 0;
}