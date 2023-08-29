#include <boost/thread/lock_guard.hpp>
#include <unordered_map>

#include <opencv2/imgproc.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>
#include <visualization_msgs/Marker.h>
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
                     table_positions_(DEFAULT_TABLE_POSITIONS),
                     has_received_map_(false)
    {
      pnh_.param("global_frame", global_frame_, std::string("map"));
      pnh_.param("tf_publish_rate", tf_publish_rate_, 10.0);
      pnh_.param("goal_distance_from_table_base_face", goal_distance_from_table_base_face_, 0.6);
      pnh_.param("hough_rho", hough_rho_, 0.01);
      pnh_.param("hough_theta", hough_theta_, M_PI / 12);
      pnh_.param("hough_threshold", hough_threshold_, 10);
      pnh_.param("hough_min_line_length", hough_min_line_length_, 0.5 * 0.75);
      pnh_.param("hough_max_line_gap", hough_max_line_gap_, 0.05);
      pnh_.param("table_base_face_center_error_tolerance",
                 table_base_face_center_error_tolerance_, 0.25);

      initializeTables();

      initializeDetectedLinesMarker();

      ros::NodeHandle nh;
      lidar_detected_lines_marker_pub_ =
          nh.advertise<visualization_msgs::Marker>("lidar/detected_lines_marker", 10);
      lidar_scan_sub_ = nh.subscribe("lidar/scan", 10, &TableManager::lidarScanCallback, this);
      map_sub_ = nh.subscribe("map", 10, &TableManager::mapCallback, this);

      static_tf_broadcaster_.sendTransform(createTableTFMsg(TableBaseID::DUAL_TABLE));

      updateReconfigurableParameters();

      reconfigure_server_.setCallback(
          boost::bind(&TableManager::reconfigureParameters, this, _1, _2));

      publish_tf_timer_ =
          pnh_.createTimer(ros::Rate(tf_publish_rate_), &TableManager::publishTF, this);
    }

  private:
    static const std::unordered_map<TableBaseID, tf2::Vector3> DEFAULT_TABLE_POSITIONS;

    static const std::vector<GoalID> TABLE_GOAL_IDS;

    void initializeTables()
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

      for (const auto &table_goal_id : TABLE_GOAL_IDS)
      {
        table_base_face_centers_.insert({table_goal_id, tf2::Vector3()});
        table_goals_.insert({table_goal_id, tf2::Transform()});
        corrected_table_goals_.insert({table_goal_id, tf2::Stamped<tf2::Transform>()});
      }

      updateTableGoals();
    }

    void updateTableGoals()
    {
      for (const auto &table_center_to_goal_quat_pair : TABLE_CENTER_TO_GOAL_QUATS)
      {
        auto &goal_id = table_center_to_goal_quat_pair.first;
        auto &table_center_to_goal_quat = table_center_to_goal_quat_pair.second;
        auto &table_base_id = GOAL_ID_TO_TABLE_BASE_ID.at(goal_id);

        table_base_face_centers_.at(goal_id) =
            table_positions_.at(table_base_id) -
            TABLE_SIZES.at(table_base_id) / 2 *
                tf2::quatRotate(table_center_to_goal_quat, {0, 1, 0});

        table_goals_.at(goal_id) =
            tf2::Transform(table_center_to_goal_quat,
                           table_base_face_centers_.at(goal_id) -
                               goal_distance_from_table_base_face_ *
                                   tf2::quatRotate(table_center_to_goal_quat, {0, 1, 0}));
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
      static ros::Time last_publish_time = ros::Time::now();

      auto now_time = ros::Time::now();
      if (last_publish_time == now_time)
      {
        return;
      }

      for (const auto &table_goal_pair : table_goals_)
      {
        auto &goal_id = table_goal_pair.first;
        auto &table_goal = table_goal_pair.second;

        if (goal_id == GoalID::MOVABLE_TABLE_1200 ||
            goal_id == GoalID::MOVABLE_TABLE_1500 ||
            goal_id == GoalID::MOVABLE_TABLE_1800)
        {
          tf_broadcaster_.sendTransform(createTableTFMsg(GOAL_ID_TO_TABLE_BASE_ID.at(goal_id)));
        }

        geometry_msgs::TransformStamped table_goal_tf_msg;
        table_goal_tf_msg.header.frame_id = global_frame_;
        table_goal_tf_msg.header.stamp = now_time;
        table_goal_tf_msg.child_frame_id = GOAL_NAMES.at(goal_id) + "_goal";
        table_goal_tf_msg.transform = tf2::toMsg(table_goal);

        tf_broadcaster_.sendTransform(table_goal_tf_msg);

        const auto &corrected_table_goal = corrected_table_goals_.at(goal_id);
        if (ros::Time::now() - corrected_table_goal.stamp_ <
            ros::Duration(ros::Rate(tf_publish_rate_)))
        {
          auto corrected_table_goal_tf_msg = tf2::toMsg(corrected_table_goal);
          corrected_table_goal_tf_msg.header.stamp = now_time;
          corrected_table_goal_tf_msg.child_frame_id =
              GOAL_NAMES.at(goal_id) + "_corrected_goal";

          tf_broadcaster_.sendTransform(corrected_table_goal_tf_msg);
        }
      }

      last_publish_time = now_time;
    }

    void initializeDetectedLinesMarker()
    {
      detected_lines_marker_.header.frame_id = global_frame_;
      detected_lines_marker_.ns = "lidar/detected_lines";
      detected_lines_marker_.id = 0;
      detected_lines_marker_.type = visualization_msgs::Marker::LINE_LIST;
      detected_lines_marker_.action = visualization_msgs::Marker::ADD;
      detected_lines_marker_.pose.orientation.w = 1.0;
      detected_lines_marker_.scale.x = 0.05;
      detected_lines_marker_.color.g = 1.0;
      detected_lines_marker_.color.a = 1.0;
    }

    void lidarScanCallback(const sensor_msgs::LaserScanConstPtr &lidar_scan)
    {
      if (!has_received_map_)
      {
        return;
      }

      tf2::Transform lidar_start_tf, lidar_end_tf;
      try
      {
        tf2::fromMsg(tf_buffer_.lookupTransform(global_frame_,
                                                lidar_scan->header.frame_id,
                                                lidar_scan->header.stamp,
                                                ros::Duration(1.0))
                         .transform,
                     lidar_start_tf);

        tf2::fromMsg(tf_buffer_.lookupTransform(global_frame_,
                                                lidar_scan->header.frame_id,
                                                lidar_scan->header.stamp +
                                                    ros::Duration(lidar_scan->ranges.size() *
                                                                  lidar_scan->time_increment),
                                                ros::Duration(1.0))
                         .transform,
                     lidar_end_tf);
      }
      catch (const tf2::TransformException &exception)
      {
        return;
      }

      cv::Mat scan_image(map_metadata_.height, map_metadata_.width, CV_8UC1, cv::Scalar(0));
      if (scan_image.empty())
      {
        return;
      }

      for (int i = 0; i < lidar_scan->ranges.size(); i++)
      {
        tf2::Vector3 local_scan_point(lidar_scan->ranges.at(i) *
                                          std::cos(lidar_scan->angle_min +
                                                   lidar_scan->angle_increment * i),
                                      lidar_scan->ranges.at(i) *
                                          std::sin(lidar_scan->angle_min +
                                                   lidar_scan->angle_increment * i),
                                      0);

        auto lerp_ratio = i / (lidar_scan->ranges.size() - 1.);
        auto global_scan_point =
            tf2::Transform(tf2::slerp(lidar_start_tf.getRotation(), lidar_end_tf.getRotation(),
                                      lerp_ratio),
                           tf2::lerp(lidar_start_tf.getOrigin(), lidar_end_tf.getOrigin(),
                                     lerp_ratio)) *
            local_scan_point;

        try
        {
          int row = static_cast<int>(
              scan_image.rows -
              std::lround((global_scan_point.y() - map_metadata_.origin.position.y) /
                          map_metadata_.resolution));
          int col = static_cast<int>(
              std::lround((global_scan_point.x() - map_metadata_.origin.position.x) /
                          map_metadata_.resolution));

          if (row >= 0 && row < scan_image.rows && col >= 0 && col < scan_image.cols)
          {
            scan_image.at<uint8_t>(row, col) = 255;
          }
          else
          {
            continue;
          }
        }
        catch (const cv::Exception &exception)
        {
          ROS_WARN("%s", exception.what());
          continue;
        }
      }

      std::vector<cv::Vec4i> lines;
      cv::HoughLinesP(scan_image, lines,
                      std::max(hough_rho_ / map_metadata_.resolution, 1.0),
                      hough_theta_,
                      hough_threshold_,
                      hough_min_line_length_ / map_metadata_.resolution,
                      hough_max_line_gap_ / map_metadata_.resolution);

      if (lines.empty())
      {
        return;
      }

      detected_lines_marker_.points.clear();

      std::vector<tf2::Vector3> line_centers;
      for (const auto &line : lines)
      {
        tf2::Vector3 line_start(
            line[0] * map_metadata_.resolution + map_metadata_.origin.position.x,
            (scan_image.rows - line[1]) * map_metadata_.resolution +
                map_metadata_.origin.position.y,
            0);
        tf2::Vector3 line_end(
            line[2] * map_metadata_.resolution + map_metadata_.origin.position.x,
            (scan_image.rows - line[3]) * map_metadata_.resolution +
                map_metadata_.origin.position.y,
            0);

        line_centers.emplace_back((line_start + line_end) / 2);

        geometry_msgs::Point line_point_msg;
        tf2::toMsg(line_start, line_point_msg);
        line_point_msg.z = lidar_start_tf.getOrigin().z();
        detected_lines_marker_.points.push_back(line_point_msg);

        tf2::toMsg(line_end, line_point_msg);
        line_point_msg.z = lidar_end_tf.getOrigin().z();
        detected_lines_marker_.points.push_back(line_point_msg);
      }

      detected_lines_marker_.header.stamp = ros::Time::now();
      lidar_detected_lines_marker_pub_.publish(detected_lines_marker_);

      for (const auto &table_base_face_center_pair : table_base_face_centers_)
      {
        auto &goal_id = table_base_face_center_pair.first;
        auto &table_base_face_center = table_base_face_center_pair.second;

        auto corrected_table_base_face_center_itr = line_centers.cend();
        for (auto line_center_itr = line_centers.cbegin();
             line_center_itr != line_centers.cend();
             line_center_itr++)
        {
          if ((*line_center_itr - table_base_face_center).length() <
              table_base_face_center_error_tolerance_)
          {
            if (corrected_table_base_face_center_itr == line_centers.cend())
            {
              corrected_table_base_face_center_itr = line_center_itr;
            }
            else
            {
              ROS_WARN("Multiple lines detected on the same table base's face.\n"
                       "Consider tuning parameters for Hough transform.");
              corrected_table_base_face_center_itr = std::min(
                  corrected_table_base_face_center_itr, line_center_itr,
                  [table_base_face_center](const auto lhs, const auto rhs)
                  {
                    return (*lhs - table_base_face_center).length() <
                           (*rhs - table_base_face_center).length();
                  });
            }
          }
        }

        if (corrected_table_base_face_center_itr != line_centers.cend())
        {
          corrected_table_goals_.at(goal_id) =
              tf2::Stamped<tf2::Transform>(
                  table_goals_.at(goal_id) *
                      tf2::Transform(tf2::Quaternion::getIdentity(),
                                     *corrected_table_base_face_center_itr -
                                         table_base_face_center),
                  ros::Time::now(), global_frame_);
        }
      }
    }

    void mapCallback(const nav_msgs::OccupancyGrid &map)
    {
      if (!has_received_map_)
      {
        has_received_map_ = true;
      }
      map_metadata_ = map.info;
    }

    void updateReconfigurableParameters()
    {
      TableManagerConfig config;
      config.goal_distance_from_table_base_face = goal_distance_from_table_base_face_;
      config.movable_table_1200_position_y =
          table_positions_.at(TableBaseID::MOVABLE_TABLE_1200).getY();
      config.movable_table_1500_position_y =
          table_positions_.at(TableBaseID::MOVABLE_TABLE_1500).getY();
      config.movable_table_1800_position_y =
          table_positions_.at(TableBaseID::MOVABLE_TABLE_1800).getY();
      config.hough_rho = hough_rho_;
      config.hough_theta = hough_theta_;
      config.hough_threshold = hough_threshold_;
      config.hough_min_line_length = hough_min_line_length_;
      config.hough_max_line_gap = hough_max_line_gap_;
      config.table_base_face_center_error_tolerance = table_base_face_center_error_tolerance_;
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
      hough_rho_ = config.hough_rho;
      hough_theta_ = config.hough_theta;
      hough_threshold_ = config.hough_threshold;
      hough_min_line_length_ = config.hough_min_line_length;
      hough_max_line_gap_ = config.hough_max_line_gap;
      table_base_face_center_error_tolerance_ = config.table_base_face_center_error_tolerance;

      updateTableGoals();
    }

    ros::NodeHandle pnh_;

    ros::Publisher lidar_detected_lines_marker_pub_;

    ros::Subscriber lidar_scan_sub_;

    ros::Subscriber map_sub_;

    ros::Timer publish_tf_timer_;

    boost::recursive_mutex reconfigure_mutex_;

    dynamic_reconfigure::Server<tsuten_behavior::TableManagerConfig> reconfigure_server_;

    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::unordered_map<TableBaseID, tf2::Vector3> table_positions_;

    std::unordered_map<GoalID, tf2::Vector3> table_base_face_centers_;

    std::unordered_map<GoalID, tf2::Transform> table_goals_;

    std::unordered_map<GoalID, tf2::Stamped<tf2::Transform>> corrected_table_goals_;

    nav_msgs::MapMetaData map_metadata_;

    visualization_msgs::Marker detected_lines_marker_;

    std::string global_frame_;

    bool has_received_map_;

    double tf_publish_rate_;

    double goal_distance_from_table_base_face_;

    double hough_rho_;
    double hough_theta_;
    int hough_threshold_;
    double hough_min_line_length_;
    double hough_max_line_gap_;

    double table_base_face_center_error_tolerance_;
  };

  const std::unordered_map<TableBaseID, tf2::Vector3> TableManager::DEFAULT_TABLE_POSITIONS =
      {{TableBaseID::DUAL_TABLE, {2.5, 2.4, 0}},
       {TableBaseID::MOVABLE_TABLE_1200, {4.5, 1.9, 0}},
       {TableBaseID::MOVABLE_TABLE_1500, {5.5, 1.9, 0}},
       {TableBaseID::MOVABLE_TABLE_1800, {6.5, 1.9, 0}}};

  const std::vector<GoalID> TableManager::TABLE_GOAL_IDS =
      {GoalID::DUAL_TABLE_F,
       GoalID::DUAL_TABLE_R,
       GoalID::DUAL_TABLE_B,
       GoalID::DUAL_TABLE_L,
       GoalID::MOVABLE_TABLE_1200,
       GoalID::MOVABLE_TABLE_1500,
       GoalID::MOVABLE_TABLE_1800};
} // namespace tsuten_behavior

int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_manager");

  tsuten_behavior::TableManager table_manager;

  ros::spin();

  return 0;
}