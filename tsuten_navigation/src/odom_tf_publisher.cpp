#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>

class TFPublisher
{
public:
  TFPublisher()
  {
    ros::NodeHandle nh(""), pnh("~");

    pnh.param("publish_rate", publish_rate_, 10.0);

    odom_sub_ = nh.subscribe("odom", 10, &TFPublisher::odomCallback, this);

    timer_ = nh.createTimer(ros::Rate(publish_rate_), &TFPublisher::publishTF, this);

    odom_.pose.pose.orientation.w = 1.0;
  }

private:
  void publishTF(const ros::TimerEvent &event)
  {
    geometry_msgs::TransformStamped odom_tf;

    odom_tf.header.stamp = ros::Time::now();
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = odom_.pose.pose.position.x;
    odom_tf.transform.translation.y = odom_.pose.pose.position.y;
    odom_tf.transform.translation.z = odom_.pose.pose.position.z;
    odom_tf.transform.rotation = odom_.pose.pose.orientation;

    tf_broadcaster_.sendTransform(odom_tf);
  }

  void odomCallback(const nav_msgs::Odometry &odom)
  {
    odom_ = odom;
  }

  ros::Subscriber odom_sub_;

  ros::Timer timer_;

  nav_msgs::Odometry odom_;

  tf2_ros::TransformBroadcaster tf_broadcaster_;

  std::string odom_topic_;

  double publish_rate_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_publisher");

  TFPublisher tf_publisher;

  ros::spin();

  return 0;
}