#include <tsuten_behavior/behavior_server.hpp>

#include <tsuten_msgs/ResetShooter.h>
#include <tsuten_msgs/ShootBottle.h>

namespace tsuten_behavior
{
  const std::array<BehaviorServer::ShooterParameter,
                   static_cast<std::size_t>(ShooterID::NUM_OF_SHOOTERS)>
      BehaviorServer::SHOOTER_PARAMETERS = {
          {{ShooterID::DUAL_TABLE_UPPER_L, "dual_table_upper_left_shooter", 0.5},
           {ShooterID::DUAL_TABLE_UPPER_R, "dual_table_upper_right_shooter", 0.5},
           {ShooterID::DUAL_TABLE_LOWER, "dual_table_lower_shooter", 0.5},
           {ShooterID::MOVABLE_TABLE_1200, "movable_table_1200_shooter", 0.5},
           {ShooterID::MOVABLE_TABLE_1500, "movable_table_1500_shooter", 0.5},
           {ShooterID::MOVABLE_TABLE_1800, "movable_table_1800_shooter", 0.5}}};

  BehaviorServer::BehaviorServer()
      : pnh_("~"),
        perform_action_server_(pnh_, "perform", false),
        move_base_action_client_("move_base"),
        is_goal_available_(false)
  {
    for (auto &shooter_param : SHOOTER_PARAMETERS)
    {
      shooters_.emplace(std::piecewise_construct,
                        std::forward_as_tuple(shooter_param.id),
                        std::forward_as_tuple(shooter_param.name,
                                              shooter_param.valve_on_duration));

      auto shoot_bottle_service_server = pnh_.advertiseService<tsuten_msgs::ShootBottleRequest,
                                                               tsuten_msgs::ShootBottleResponse>(
          shooter_param.name + "/shoot_bottle",
          [this, shooter_param](auto &, auto &)
          { shooters_.at(shooter_param.id).shootBottle(); return true; });

      shoot_bottle_service_servers_.insert({shooter_param.id, shoot_bottle_service_server});
    }

    shoot_on_table_service_server_ = pnh_.advertiseService(
        "shoot_on_table", &BehaviorServer::shootOnTable, this);

    reset_shooter_service_server_ = pnh_.advertiseService<tsuten_msgs::ResetShooterRequest,
                                                          tsuten_msgs::ResetShooterResponse>(
        "reset_all_shooters",
        [this](auto &, auto &)
        { resetAllShooters(); return true; });

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