#include <ros/ros.h>
#include <ros/package.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <tsuten_behavior/constants.hpp>

namespace tsuten_behavior
{
  class BehaviorVisualizer
  {
  public:
    BehaviorVisualizer()
    {
      ros::NodeHandle pnh("~");

      pnh.param("global_frame", global_frame_, std::string("map"));
      pnh.param("table_markers_publish_rate", table_markers_publish_rate_, 10.0);

      table_markers_pub_ =
          pnh.advertise<visualization_msgs::MarkerArray>("table_markers", 10, true);

      publish_table_markers_timer_ =
          pnh.createTimer(ros::Rate(table_markers_publish_rate_),
                          &BehaviorVisualizer::publishTableMarkers, this);

      createTableMarkers();
    }

  private:
    static const std::unordered_map<TableID, std::array<float, 4>>
        TABLE_MARKER_COLORS;

    void createTableMarkers()
    {
      static const std::string table_mesh_resource_dir =
          ros::package::getPath("tsuten_description") + "/meshes";

      for (auto &table_name_pair : TABLE_NAMES)
      {
        auto &table_id = table_name_pair.first;
        auto table_name = table_name_pair.second;

        if (table_id == TableID::DUAL_TABLE_LOWER || table_id == TableID::DUAL_TABLE_UPPER)
        {
          continue;
        }

        visualization_msgs::Marker table_marker;
        table_marker.header.frame_id = global_frame_;
        table_marker.ns = table_name;
        table_marker.id = static_cast<int32_t>(table_id);
        table_marker.type = visualization_msgs::Marker::CUBE;
        // table_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        // table_marker.mesh_resource = table_mesh_resource_dir + "/" + table_name + ".stl";
        table_marker.scale.x = TABLE_SIZES.at(table_id).at(0);
        table_marker.scale.y = TABLE_SIZES.at(table_id).at(1);
        table_marker.scale.z = TABLE_SIZES.at(table_id).at(2);
        table_marker.color.r = TABLE_MARKER_COLORS.at(table_id).at(0);
        table_marker.color.g = TABLE_MARKER_COLORS.at(table_id).at(1);
        table_marker.color.b = TABLE_MARKER_COLORS.at(table_id).at(2);
        table_marker.color.a = TABLE_MARKER_COLORS.at(table_id).at(3);

        table_markers_.insert({table_id, table_marker});
      }
    }

    void publishTableMarkers(const ros::TimerEvent &event)
    {
      visualization_msgs::MarkerArray table_markers_msg;

      for (auto &table_marker_pair : table_markers_)
      {
        auto &table_id = table_marker_pair.first;
        auto &table_marker = table_marker_pair.second;

        table_marker.header.stamp = ros::Time::now();

        tf2::Transform tf;
        if (getTF(table_id, tf))
        {
          tf2::toMsg(tf, table_marker.pose);
          table_markers_msg.markers.push_back(table_marker);
        }
      }

      table_markers_pub_.publish(table_markers_msg);
    }

    bool getTF(TableID table_id, tf2::Transform &tf)
    {
      try
      {
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);
        tf2::fromMsg(tf_buffer.lookupTransform(
                                  global_frame_, TABLE_NAMES.at(table_id) + "_link",
                                  ros::Time(0), ros::Duration(5.0))
                         .transform,
                     tf);

        return true;
      }
      catch (const tf2::TransformException &exception)
      {
        ROS_ERROR("%s", exception.what());

        return false;
      }
    }

    ros::Publisher table_markers_pub_;

    ros::Timer publish_table_markers_timer_;

    std::unordered_map<TableID, visualization_msgs::Marker> table_markers_;

    std::string global_frame_;

    double table_markers_publish_rate_;
  };

  const std::unordered_map<TableID, std::array<float, 4>>
      BehaviorVisualizer::TABLE_MARKER_COLORS = {
          {TableID::DUAL_TABLE, {1.0, 0.0, 0.0, 1.0}},
          {TableID::MOVABLE_TABLE_1200, {1.0, 1.0, 0.0, 1.0}},
          {TableID::MOVABLE_TABLE_1500, {1.0, 1.0, 0.0, 1.0}},
          {TableID::MOVABLE_TABLE_1800, {1.0, 1.0, 0.0, 1.0}}};
} // namespace tsuten_behavior

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_visualizer");

  tsuten_behavior::BehaviorVisualizer behavior_visualizer;

  ros::spin();

  return 0;
}