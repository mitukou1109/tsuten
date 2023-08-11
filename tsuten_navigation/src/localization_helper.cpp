#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tsuten_msgs/CorrectRobotPose.h>

namespace tsuten_navigation
{
  class LocalizationHandler
  {
  public:
    LocalizationHandler()
    {
      ros::NodeHandle nh(""), pnh("~");

      pnh.param("global_frame", global_frame_, std::string("map"));
      pnh.param("odom_frame", odom_frame_, std::string("odom"));
      pnh.param("odom_topic", odom_topic_, std::string("odom"));
      pnh.param("publish_rate", publish_rate_, 20.0);
      pnh.param("publish_odom_tf", publish_odom_tf_, true);

      odom_sub_ = nh.subscribe(odom_topic_, 10, &LocalizationHandler::odomCallback, this);

      timer_ = nh.createTimer(ros::Rate(publish_rate_),
                              &LocalizationHandler::publishTF, this);

      correct_robot_pose_service_server_ = nh.advertiseService(
          "correct_robot_pose", &LocalizationHandler::correctRobotPose, this);

      odom_to_global_tf_.setIdentity();
      robot_base_to_odom_tf_.setIdentity();
    }

  private:
    void publishTF(const ros::TimerEvent &event)
    {
      static ros::Time last_publish_time = ros::Time::now();

      auto now_time = ros::Time::now();
      if (last_publish_time == now_time)
      {
        return;
      }

      geometry_msgs::TransformStamped odom_to_global_tf_msg;
      odom_to_global_tf_msg.header.stamp = now_time + ros::Duration(0.05);
      odom_to_global_tf_msg.header.frame_id = global_frame_;
      odom_to_global_tf_msg.child_frame_id = odom_frame_;
      odom_to_global_tf_msg.transform = tf2::toMsg(odom_to_global_tf_);

      tf_broadcaster_.sendTransform(odom_to_global_tf_msg);

      if (publish_odom_tf_)
      {
        geometry_msgs::TransformStamped robot_base_to_odom_tf_msg;
        robot_base_to_odom_tf_msg.header.stamp = now_time;
        robot_base_to_odom_tf_msg.header.frame_id = odom_frame_;
        robot_base_to_odom_tf_msg.child_frame_id = robot_base_frame_;
        robot_base_to_odom_tf_msg.transform = tf2::toMsg(robot_base_to_odom_tf_);

        tf_broadcaster_.sendTransform(robot_base_to_odom_tf_msg);
      }

      last_publish_time = now_time;
    }

    void odomCallback(const nav_msgs::Odometry &odom)
    {
      tf2::fromMsg(odom.pose.pose, robot_base_to_odom_tf_);
      robot_base_frame_ = odom.child_frame_id;
    }

    bool correctRobotPose(tsuten_msgs::CorrectRobotPoseRequest &req,
                          tsuten_msgs::CorrectRobotPoseResponse &res)
    {
      if (req.robot_pose.header.frame_id != global_frame_)
      {
        ROS_ERROR("Frame ID [%s] does not match the global frame ID [%s]",
                  req.robot_pose.header.frame_id.c_str(), global_frame_.c_str());
        return false;
      }

      tf2::Transform new_robot_base_to_global_tf;
      tf2::fromMsg(req.robot_pose.pose, new_robot_base_to_global_tf);

      odom_to_global_tf_ = robot_base_to_odom_tf_.inverse() * new_robot_base_to_global_tf;

      return true;
    }

    ros::Subscriber odom_sub_;

    ros::Timer timer_;

    ros::ServiceServer correct_robot_pose_service_server_;

    tf2::Transform odom_to_global_tf_;
    tf2::Transform robot_base_to_odom_tf_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string global_frame_;
    std::string odom_frame_;
    std::string robot_base_frame_;

    std::string odom_topic_;

    double publish_rate_;

    bool publish_odom_tf_;
  };
} // namespace tsuten_navigation

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localization_handler");

  tsuten_navigation::LocalizationHandler localization_handler;

  ros::spin();

  return 0;
}