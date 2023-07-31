#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

#include <tsuten_msgs/CorrectRobotPosition.h>

namespace tsuten_navigation
{
  class OdomTFPublisher
  {
  public:
    OdomTFPublisher()
    {
      ros::NodeHandle nh(""), pnh("~");

      pnh.param("global_frame", global_frame_, std::string("map"));
      pnh.param("odom_frame", odom_frame_, std::string("odom"));
      pnh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));
      pnh.param("odom_topic", odom_topic_, std::string("odom"));
      pnh.param("publish_rate", publish_rate_, 10.0);

      odom_sub_ = nh.subscribe(odom_topic_, 10, &OdomTFPublisher::odomCallback, this);

      timer_ = nh.createTimer(ros::Rate(publish_rate_), &OdomTFPublisher::publishOdomTF, this);

      correct_robot_position_service_server_ = nh.advertiseService(
          "correct_robot_position", &OdomTFPublisher::correctRobotPosition, this);

      map_tf_.setIdentity();
      odom_tf_.setIdentity();
    }

  private:
    void publishOdomTF(const ros::TimerEvent &event)
    {
      geometry_msgs::TransformStamped map_tf_msg;
      map_tf_msg.header.stamp = ros::Time::now();
      map_tf_msg.header.frame_id = global_frame_;
      map_tf_msg.child_frame_id = odom_frame_;
      map_tf_msg.transform = tf2::toMsg(map_tf_);

      tf_broadcaster_.sendTransform(map_tf_msg);

      geometry_msgs::TransformStamped odom_tf_msg;
      odom_tf_msg.header.stamp = ros::Time::now();
      odom_tf_msg.header.frame_id = odom_frame_;
      odom_tf_msg.child_frame_id = robot_base_frame_;
      odom_tf_msg.transform = tf2::toMsg(odom_tf_);

      tf_broadcaster_.sendTransform(odom_tf_msg);
    }

    void odomCallback(const nav_msgs::Odometry &odom)
    {
      tf2::fromMsg(odom.pose.pose, odom_tf_);
    }

    bool correctRobotPosition(tsuten_msgs::CorrectRobotPositionRequest &req,
                              tsuten_msgs::CorrectRobotPositionResponse &res)
    {
      tf2::Vector3 new_robot_position;
      tf2::fromMsg(req.robot_position, new_robot_position);
      tf2::Transform new_robot_position_tf(tf2::Quaternion::getIdentity(), new_robot_position);

      map_tf_ = new_robot_position_tf * odom_tf_.inverse();

      return true;
    }

    ros::Subscriber odom_sub_;

    ros::Timer timer_;

    ros::ServiceServer correct_robot_position_service_server_;

    tf2::Transform map_tf_;
    tf2::Transform odom_tf_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::string global_frame_;
    std::string odom_frame_;
    std::string robot_base_frame_;

    std::string odom_topic_;

    double publish_rate_;
  };
} // namespace tsuten_navigation

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_tf_publisher");

  tsuten_navigation::OdomTFPublisher odom_tf_publisher;

  ros::spin();

  return 0;
}