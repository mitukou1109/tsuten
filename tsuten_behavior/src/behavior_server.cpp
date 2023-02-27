#include <tsuten_behavior/behavior_server.hpp>

#include <XmlRpcException.h>

#include <tsuten_msgs/ResetShooter.h>
#include <tsuten_msgs/ShootBottle.h>

namespace tsuten_behavior
{
  const std::unordered_map<TableID, tf2::Transform> BehaviorServer::DEFAULT_TABLE_TFS = {
      {TableID::DUAL_TABLE, tf2::Transform({{0, 0, 1}, M_PI / 2}, {2.5, 2.4, 0})},
      {TableID::MOVABLE_TABLE_1200, tf2::Transform({{0, 0, 1}, M_PI / 2}, {4.5, 1.9, 0})},
      {TableID::MOVABLE_TABLE_1500, tf2::Transform({{0, 0, 1}, M_PI / 2}, {4.5, 1.9, 0})},
      {TableID::MOVABLE_TABLE_1800, tf2::Transform({{0, 0, 1}, M_PI / 2}, {6.5, 1.9, 0})}};

  const std::unordered_map<ShooterID, BehaviorServer::ShooterParameter>
      BehaviorServer::SHOOTER_PARAMETERS = {
          {{ShooterID::DUAL_TABLE_LOWER, {"dual_table_lower_shooter", 0.5}},
           {ShooterID::DUAL_TABLE_UPPER_L, {"dual_table_upper_left_shooter", 0.5}},
           {ShooterID::DUAL_TABLE_UPPER_R, {"dual_table_upper_right_shooter", 0.5}},
           {ShooterID::MOVABLE_TABLE_1200, {"movable_table_1200_shooter", 0.5}},
           {ShooterID::MOVABLE_TABLE_1500, {"movable_table_1500_shooter", 0.5}},
           {ShooterID::MOVABLE_TABLE_1800, {"movable_table_1800_shooter", 0.5}}}};

  BehaviorServer::BehaviorServer()
      : pnh_("~"),
        table_tfs_(DEFAULT_TABLE_TFS),
        perform_action_server_(pnh_, "perform", false),
        move_base_action_client_("move_base"),
        is_goal_available_(false)
  {
    initializeTableTFs();

    for (auto &table_tf_pair : table_tfs_)
    {
      auto &table_id = table_tf_pair.first;
      auto &table_position = table_tf_pair.second.getOrigin();

      auto shoot_bottle_service_server = pnh_.advertiseService<tsuten_msgs::ShootBottleRequest,
                                                               tsuten_msgs::ShootBottleResponse>(
          shooter_param.name + "/shoot_bottle",
          [this, shooter_param](auto &, auto &)
          { shooters_.at(shooter_param.id).shootBottle(); return true; });

      shoot_bottle_service_servers_.insert({shooter_param.id, shoot_bottle_service_server});
    }

    pnh_.param("global_frame", global_frame_, std::string("map"));
    pnh_.param("tf_publish_rate", tf_publish_rate_, 10.0);

    static_tf_broadcaster_.sendTransform(createTableTFMsg(TableID::DUAL_TABLE));

    publish_tf_timer_ = pnh_.createTimer(ros::Rate(tf_publish_rate_), &BehaviorServer::publishTF, this);

    reset_shooter_service_server_ = pnh_.advertiseService<tsuten_msgs::ResetShooterRequest,
                                                          tsuten_msgs::ResetShooterResponse>(
        "reset_all_shooters",
        [this](auto &, auto &)
        { resetAllShooters(); return true; });

    shoot_on_table_service_server_ = pnh_.advertiseService(
        "shoot_on_table", &BehaviorServer::shootOnTable, this);

    for (auto &shooter_parameter_pair : SHOOTER_PARAMETERS)
    {
      auto &shooter_id = shooter_parameter_pair.first;
      auto &shooter_parameter = shooter_parameter_pair.second;

      shooters_.emplace(std::piecewise_construct,
                        std::forward_as_tuple(shooter_id),
                        std::forward_as_tuple(shooter_parameter.name,
                                              shooter_parameter.valve_on_duration));

      auto shoot_bottle_service_server = pnh_.advertiseService<tsuten_msgs::ShootBottleRequest,
                                                               tsuten_msgs::ShootBottleResponse>(
          shooter_parameter.name + "/shoot_bottle",
          [this, shooter_id](auto &, auto &)
          { shooters_.at(shooter_id).shootBottle(); return true; });

      shoot_bottle_service_servers_.insert({shooter_id, shoot_bottle_service_server});
    }

    perform_action_server_.registerGoalCallback(
        std::bind(&BehaviorServer::acceptPerformGoal, this));

    perform_action_server_.registerPreemptCallback(
        std::bind(&BehaviorServer::preemptPerformAction, this));

    perform_action_server_.start();

    if (!move_base_action_client_.waitForServer(ros::Duration(3.0)))
    {
      ROS_ERROR("move_base action server timeout");
    }

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
      shooters_.at(ShooterID::DUAL_TABLE_UPPER_L).shootBottle();
      boost::this_thread::sleep_for(boost::chrono::milliseconds(1000));
      shooters_.at(ShooterID::DUAL_TABLE_UPPER_R).shootBottle();
    }

    if (isDirectedToPerformAt(tsuten_msgs::PerformGoal::DUAL_TABLE_LOWER))
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(3000));
      shooters_.at(ShooterID::DUAL_TABLE_LOWER).shootBottle();
    }

    if (isDirectedToPerformAt(tsuten_msgs::PerformGoal::MOVABLE_TABLE_1200))
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(3000));
      shooters_.at(ShooterID::MOVABLE_TABLE_1200).shootBottle();
    }

    if (isDirectedToPerformAt(tsuten_msgs::PerformGoal::MOVABLE_TABLE_1500))
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(3000));
      shooters_.at(ShooterID::MOVABLE_TABLE_1500).shootBottle();
    }

    if (isDirectedToPerformAt(tsuten_msgs::PerformGoal::MOVABLE_TABLE_1800))
    {
      boost::this_thread::sleep_for(boost::chrono::milliseconds(3000));
      shooters_.at(ShooterID::MOVABLE_TABLE_1800).shootBottle();
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

      is_goal_available_ = false;
    }
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
    ros::NodeHandle nh;

    XmlRpc::XmlRpcValue table_position_list;
    if (nh.getParam("table_position", table_position_list))
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
            ROS_ERROR("Invalid table name: %s", table_name.c_str());
          }
        }
        else
        {
          ROS_ERROR("Invalid table position parameter: %s",
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
} // namespace tsuten_behavior

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_server");

  tsuten_behavior::BehaviorServer behavior_server;

  ros::spin();

  return 0;
}