#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

namespace tsuten_navigation
{
  class OdomTFPublisher
  {
  public:
    OdomTFPublisher()
    {
      ros::NodeHandle nh(""), pnh("~");

      pnh.param("odom_topic", odom_topic_, std::string("odom"));
      pnh.param("publish_rate", publish_rate_, 10.0);

      odom_sub_ = nh.subscribe(odom_topic_, 10, &OdomTFPublisher::odomCallback, this);

      timer_ = nh.createTimer(ros::Rate(publish_rate_), &OdomTFPublisher::publishOdomTF, this);

      odom_.pose.pose.orientation.w = 1.0;
    }

  private:
    void publishOdomTF(const ros::TimerEvent &event)
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
} // namespace tsuten_navigation

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_tf_publisher");

  tsuten_navigation::OdomTFPublisher odom_tf_publisher;

  ros::spin();

  return 0;
}