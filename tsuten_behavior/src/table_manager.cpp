#include <algorithm>
#include <boost/thread/lock_guard.hpp>
#include <random>
#include <unordered_map>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/PointStamped.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <XmlRpcException.h>

#include <tsuten_behavior/constants.hpp>
#include <tsuten_msgs/TableManagerConfig.h>

namespace tsuten_behavior
{
  class TableManager
  {
  public:
    TableManager() : pnh_("~"),
                     tf_listener_(tf_buffer_),
                     reconfigure_server_(reconfigure_mutex_),
                     table_positions_(DEFAULT_TABLE_POSITIONS)
    {
      pnh_.param("global_frame", global_frame_, std::string("map"));
      pnh_.param("tf_publish_rate", tf_publish_rate_, 10.0);
      pnh_.param("cluster_tolerance", cluster_tolerance_, 0.1);
      pnh_.param("min_cluster_size", min_cluster_size_, 5);
      pnh_.param("table_pole_error_tolerance", table_pole_error_tolerance, 0.25);

      ros::NodeHandle nh;
      lidar_scan_sub_ = nh.subscribe("lidar/scan", 10, &TableManager::lidarScanCallback, this);
      for (auto &table_base_name_pair : TABLE_BASE_NAMES)
      {
        auto &table_base_id = table_base_name_pair.first;
        auto &table_base_name = table_base_name_pair.second;

        table_pole_pubs_.insert(
            {table_base_id,
             nh.advertise<geometry_msgs::PointStamped>("table_pole/" + table_base_name, 10)});
      }

      initializeTableTFs();

      static_tf_broadcaster_.sendTransform(createTableTFMsg(TableBaseID::DUAL_TABLE));

      updateReconfigurableParameters();

      reconfigure_server_.setCallback(
          boost::bind(&TableManager::reconfigureParameters, this, _1, _2));

      publish_tf_timer_ =
          pnh_.createTimer(ros::Rate(tf_publish_rate_), &TableManager::publishTF, this);
    }

  private:
    static const std::unordered_map<TableBaseID, tf2::Vector3> DEFAULT_TABLE_POSITIONS;

    void initializeTableTFs()
    {
      XmlRpc::XmlRpcValue table_position_list;
      if (pnh_.getParam("table_position", table_position_list))
      {
        for (auto &table_position_pair : table_position_list)
        {
          auto &table_base_name = table_position_pair.first;
          auto &table_position = table_position_pair.second;

          if (table_position.getType() == XmlRpc::XmlRpcValue::TypeArray)
          {
            auto table_base_name_pair_itr = std::find_if(
                TABLE_BASE_NAMES.cbegin(), TABLE_BASE_NAMES.cend(),
                [&table_base_name](const std::pair<TableBaseID, std::string> &table_base_name_pair)
                { return table_base_name == table_base_name_pair.second; });

            if (table_base_name_pair_itr != TABLE_BASE_NAMES.cend())
            {
              auto &table_base_id = (*table_base_name_pair_itr).first;
              try
              {
                table_positions_.at(table_base_id) = {static_cast<double>(table_position[0]),
                                                      static_cast<double>(table_position[1]),
                                                      0};
              }
              catch (const XmlRpc::XmlRpcException &exception)
              {
                ROS_ERROR(
                    "Error parsing %s position: %s. Make sure to set parameter in double type.\n"
                    "Using default values.",
                    table_base_name.c_str(), exception.getMessage().c_str());
              }
            }
            else
            {
              ROS_ERROR("Invalid table_name: %s", table_base_name.c_str());
            }
          }
          else
          {
            ROS_ERROR("Invalid %s position parameter: %s",
                      table_base_name.c_str(),
                      static_cast<std::string>(table_position).c_str());
          }
        }
      }
      else
      {
        ROS_WARN("Table positions not set, using default values");
      }
    }

    geometry_msgs::TransformStamped createTableTFMsg(const TableBaseID &table_base_id)
    {
      geometry_msgs::TransformStamped table_tf_msg;
      table_tf_msg.header.frame_id = global_frame_;
      table_tf_msg.header.stamp = ros::Time::now();
      table_tf_msg.child_frame_id = TABLE_BASE_NAMES.at(table_base_id) + "_link";
      table_tf_msg.transform.translation = tf2::toMsg(table_positions_.at(table_base_id));
      table_tf_msg.transform.rotation = tf2::toMsg(tf2::Quaternion::getIdentity());

      return table_tf_msg;
    }

    void publishTF(const ros::TimerEvent &event)
    {
      for (auto &table_position_pair : table_positions_)
      {
        tf_broadcaster_.sendTransform(createTableTFMsg(table_position_pair.first));
      }
    }

    void lidarScanCallback(const sensor_msgs::LaserScanConstPtr &lidar_scan)
    {
      static std::mt19937 random_engine((std::random_device())());
      static const int NUM_OF_CLUSTER_REP_POINTS = 3;
      static const int NUM_OF_ARC_CENTER_SAMPLES = 10;
      static const double ARC_CENTER_ERROR_TOLERANCE = 0.05;

      if (!tf_buffer_.canTransform(
              lidar_scan->header.frame_id,
              global_frame_,
              lidar_scan->header.stamp +
                  ros::Duration().fromSec(lidar_scan->ranges.size() * lidar_scan->time_increment),
              ros::Duration(1.0)))
      {
        return;
      }

      sensor_msgs::PointCloud2 cloud_msg;
      laser_projector_.transformLaserScanToPointCloud(global_frame_, *lidar_scan,
                                                      cloud_msg, tf_buffer_);

      auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::fromROSMsg(cloud_msg, *cloud);

      auto kd_tree = boost::make_shared<pcl::search::KdTree<pcl::PointXYZ>>();
      kd_tree->setInputCloud(cloud);

      pcl::EuclideanClusterExtraction<pcl::PointXYZ> cluster_extractor;
      cluster_extractor.setClusterTolerance(cluster_tolerance_);
      cluster_extractor.setMinClusterSize(min_cluster_size_);
      cluster_extractor.setSearchMethod(kd_tree);
      cluster_extractor.setInputCloud(cloud);

      std::vector<pcl::PointIndices> clusters;
      cluster_extractor.extract(clusters);

      for (auto &cluster : clusters)
      {
        std::vector<tf2::Vector3> arc_center_samples;
        for (int i = 0; i < NUM_OF_ARC_CENTER_SAMPLES; i++)
        {
          std::vector<int> rep_indices;
          std::sample(cluster.indices.cbegin(), cluster.indices.cend(),
                      std::back_inserter(rep_indices), NUM_OF_CLUSTER_REP_POINTS, random_engine);

          std::array<pcl::PointXYZ, NUM_OF_CLUSTER_REP_POINTS> p;
          for (int i = 0; i < NUM_OF_CLUSTER_REP_POINTS; i++)
          {
            p.at(i) = cloud->points.at(rep_indices.at(i));
          }

          double d = 2 * (p[0].x - p[1].x) * (p[0].y - p[2].y) -
                     2 * (p[0].x - p[2].x) * (p[0].y - p[1].y);

          double n_x = (p[0].y - p[1].y) * (std::pow(p[2].x, 2) - std::pow(p[0].x, 2) +
                                            std::pow(p[2].y, 2) - std::pow(p[0].y, 2)) -
                       (p[0].y - p[2].y) * (std::pow(p[1].x, 2) - std::pow(p[0].x, 2) +
                                            std::pow(p[1].y, 2) - std::pow(p[0].y, 2));

          double n_y = (p[0].x - p[2].x) * (std::pow(p[1].x, 2) - std::pow(p[0].x, 2) +
                                            std::pow(p[1].y, 2) - std::pow(p[0].y, 2)) -
                       (p[0].x - p[1].x) * (std::pow(p[2].x, 2) - std::pow(p[0].x, 2) +
                                            std::pow(p[2].y, 2) - std::pow(p[0].y, 2));

          arc_center_samples.emplace_back(n_x / d, n_y / d, 0);
        }

        auto arc_center_avg =
            std::accumulate(arc_center_samples.cbegin(), arc_center_samples.cend(),
                            tf2::Vector3(0, 0, 0)) /
            arc_center_samples.size();

        for (auto arc_center_itr = arc_center_samples.cbegin();
             arc_center_itr != arc_center_samples.cend();)
        {
          if ((*arc_center_itr - arc_center_avg).length() > ARC_CENTER_ERROR_TOLERANCE)
          {
            arc_center_itr = arc_center_samples.erase(arc_center_itr);
          }
          else
          {
            arc_center_itr++;
          }
        }

        arc_center_avg =
            std::accumulate(arc_center_samples.cbegin(), arc_center_samples.cend(),
                            tf2::Vector3(0, 0, 0)) /
            arc_center_samples.size();

        for (auto &table_position_pair : table_positions_)
        {
          auto &table_base_id = table_position_pair.first;
          auto &table_position = table_position_pair.second;

          if ((arc_center_avg - table_position).length() < table_pole_error_tolerance)
          {
            geometry_msgs::PointStamped table_pole_msg;
            table_pole_msg.header.frame_id = global_frame_;
            table_pole_msg.header.stamp = ros::Time::now();
            tf2::toMsg(arc_center_avg, table_pole_msg.point);
            table_pole_msg.point.z = cloud->points.at(0).z;

            table_pole_pubs_.at(table_base_id).publish(table_pole_msg);
          }
        }
      }
    }

    void updateReconfigurableParameters()
    {
      TableManagerConfig config;
      config.movable_table_1200_position_y =
          table_positions_.at(TableBaseID::MOVABLE_TABLE_1200).getY();
      config.movable_table_1500_position_y =
          table_positions_.at(TableBaseID::MOVABLE_TABLE_1500).getY();
      config.movable_table_1800_position_y =
          table_positions_.at(TableBaseID::MOVABLE_TABLE_1800).getY();
      {
        boost::lock_guard<boost::recursive_mutex> lock(reconfigure_mutex_);
        reconfigure_server_.updateConfig(config);
      }
    }

    void reconfigureParameters(tsuten_behavior::TableManagerConfig &config, uint32_t level)
    {
      table_positions_.at(TableBaseID::MOVABLE_TABLE_1200)
          .setY(config.movable_table_1200_position_y);
      table_positions_.at(TableBaseID::MOVABLE_TABLE_1500)
          .setY(config.movable_table_1500_position_y);
      table_positions_.at(TableBaseID::MOVABLE_TABLE_1800)
          .setY(config.movable_table_1800_position_y);
    }

    ros::NodeHandle pnh_;

    std::unordered_map<TableBaseID, ros::Publisher> table_pole_pubs_;

    ros::Subscriber lidar_scan_sub_;

    ros::Timer publish_tf_timer_;

    boost::recursive_mutex reconfigure_mutex_;

    dynamic_reconfigure::Server<tsuten_behavior::TableManagerConfig> reconfigure_server_;

    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::unordered_map<TableBaseID, tf2::Vector3> table_positions_;

    laser_geometry::LaserProjection laser_projector_;

    std::string global_frame_;

    double tf_publish_rate_;

    double cluster_tolerance_;
    int min_cluster_size_;

    double table_pole_error_tolerance;
  };

  const std::unordered_map<TableBaseID, tf2::Vector3> TableManager::DEFAULT_TABLE_POSITIONS =
      {{TableBaseID::DUAL_TABLE, {2.5, 2.4, 0}},
       {TableBaseID::MOVABLE_TABLE_1200, {4.5, 1.9, 0}},
       {TableBaseID::MOVABLE_TABLE_1500, {5.5, 1.9, 0}},
       {TableBaseID::MOVABLE_TABLE_1800, {6.5, 1.9, 0}}};
} // namespace tsuten_behavior

int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_manager");

  tsuten_behavior::TableManager table_manager;

  ros::spin();

  return 0;
}