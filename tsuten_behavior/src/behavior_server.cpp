#include <unordered_map>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <tsuten_mechanism/bottle_shooter_controller.hpp>
#include <tsuten_msgs/PerformAction.h>

class BehaviorServer
{
public:
  BehaviorServer()
      : pnh_("~"),
        perform_action_server_(pnh_, "perform", false),
        move_base_action_client_("move_base"),
        is_goal_available_(false)
  {
    for (auto &shooter_param : SHOOTER_PARAMETERS)
    {
      shooters_.emplace(std::piecewise_construct,
                        std::forward_as_tuple(shooter_param.id),
                        std::forward_as_tuple(
                            shooter_param.name, shooter_param.valve_on_duration));
    }

    if (!move_base_action_client_.waitForServer(ros::Duration(5.0)))
    {
      ROS_ERROR("move_base action server timeout");
    }

    perform_action_server_.registerGoalCallback(
        std::bind(&BehaviorServer::performActionGoalCB, this));
    perform_action_server_.registerPreemptCallback(
        std::bind(&BehaviorServer::performActionPreemptCB, this));
    perform_action_server_.start();

    launch_perform_thread_ = std::make_unique<boost::thread>([this]
                                                             { launchPerformThread(); });
  }

  ~BehaviorServer()
  {
    perform_thread_->interrupt();
    launch_perform_thread_->interrupt();
  }

private:
  void performThread()
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

  void launchPerformThread()
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

  void performActionGoalCB()
  {
    {
      boost::lock_guard<boost::mutex> lock(mutex_);

      perform_goal_ = perform_action_server_.acceptNewGoal();
      is_goal_available_ = true;
    }
    is_goal_available_cv_.notify_all();
  }

  void performActionPreemptCB()
  {
    {
      boost::lock_guard<boost::mutex> lock(mutex_);
      is_goal_available_ = false;
    }

    perform_thread_->interrupt();

    perform_action_server_.setPreempted();
    ROS_INFO("Performance canceled");

    resetShooters();
  }

  void resetShooters()
  {
    for (auto &shooter : shooters_)
    {
      shooter.second.resetShooter();
    }
  }

  enum class ShooterID : std::size_t
  {
    DUAL_TABLE_UPPER_L,
    DUAL_TABLE_UPPER_R,
    DUAL_TABLE_LOWER,
    MOVABLE_TABLE_1200,
    MOVABLE_TABLE_1500,
    MOVABLE_TABLE_1800,
    NUM_OF_SHOOTERS
  };

  struct ShooterParameter
  {
    ShooterID id;
    std::string name;
    double valve_on_duration;
  };

  const std::array<ShooterParameter, static_cast<std::size_t>(ShooterID::NUM_OF_SHOOTERS)>
      SHOOTER_PARAMETERS = {
          {{ShooterID::DUAL_TABLE_UPPER_L, "dual_table_upper_left_shooter", 0.5},
           {ShooterID::DUAL_TABLE_UPPER_R, "dual_table_upper_right_shooter", 0.5},
           {ShooterID::DUAL_TABLE_LOWER, "dual_table_lower_shooter", 0.5},
           {ShooterID::MOVABLE_TABLE_1200, "movable_table_1200_shooter", 0.5},
           {ShooterID::MOVABLE_TABLE_1500, "movable_table_1500_shooter", 0.5},
           {ShooterID::MOVABLE_TABLE_1800, "movable_table_1800_shooter", 0.5}}};

  ros::NodeHandle pnh_;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_action_client_;

  actionlib::SimpleActionServer<tsuten_msgs::PerformAction> perform_action_server_;

  std::unordered_map<ShooterID, BottleShooterController> shooters_;

  tsuten_msgs::PerformGoalConstPtr perform_goal_;

  boost::condition_variable is_goal_available_cv_;

  bool is_goal_available_;

  boost::mutex mutex_;

  std::unique_ptr<boost::thread> launch_perform_thread_;
  std::unique_ptr<boost::thread> perform_thread_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_server");

  BehaviorServer behavior_server;

  ros::spin();

  return 0;
}