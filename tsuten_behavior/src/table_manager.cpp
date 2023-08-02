#include <boost/thread/lock_guard.hpp>
#include <unordered_map>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
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
                     reconfigure_server_(reconfigure_mutex_),
                     table_tfs_(DEFAULT_TABLE_TFS)
    {
      pnh_.param("global_frame", global_frame_, std::string("map"));
      pnh_.param("tf_publish_rate", tf_publish_rate_, 10.0);

      initializeTableTFs();

      static_tf_broadcaster_.sendTransform(createTableTFMsg(TableID::DUAL_TABLE));

      updateReconfigurableParameters();

      reconfigure_server_.setCallback(
          boost::bind(&TableManager::reconfigureParameters, this, _1, _2));

      publish_tf_timer_ =
          pnh_.createTimer(ros::Rate(tf_publish_rate_), &TableManager::publishTF, this);
    }

  private:
    static const std::unordered_map<TableID, tf2::Transform> DEFAULT_TABLE_TFS;

    void initializeTableTFs()
    {
      XmlRpc::XmlRpcValue table_position_list;
      if (pnh_.getParam("table_position", table_position_list))
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
              ROS_ERROR("Invalid table_name: %s", table_name.c_str());
            }
          }
          else
          {
            ROS_ERROR("Invalid %s position parameter: %s",
                      table_name.c_str(),
                      static_cast<std::string>(table_position).c_str());
          }
        }
      }
      else
      {
        ROS_WARN("Table positions not set, using default values");
      }
    }

    geometry_msgs::TransformStamped createTableTFMsg(TableID table_id)
    {
      geometry_msgs::TransformStamped table_tf_msg;
      table_tf_msg.header.frame_id = global_frame_;
      table_tf_msg.header.stamp = ros::Time::now();
      table_tf_msg.child_frame_id = TABLE_NAMES.at(table_id) + "_link";
      table_tf_msg.transform = tf2::toMsg(table_tfs_.at(table_id));

      return table_tf_msg;
    }

    void publishTF(const ros::TimerEvent &event)
    {
      for (auto &table_tf_pair : table_tfs_)
      {
        auto &table_id = table_tf_pair.first;

        if (table_id != TableID::DUAL_TABLE)
        {
          tf_broadcaster_.sendTransform(createTableTFMsg(table_id));
        }
      }
    }

    void updateReconfigurableParameters()
    {
      TableManagerConfig config;
      config.movable_table_1200_position_y = table_tfs_.at(TableID::MOVABLE_TABLE_1200)
                                                 .getOrigin()
                                                 .getY();
      config.movable_table_1500_position_y = table_tfs_.at(TableID::MOVABLE_TABLE_1500)
                                                 .getOrigin()
                                                 .getY();
      config.movable_table_1800_position_y = table_tfs_.at(TableID::MOVABLE_TABLE_1800)
                                                 .getOrigin()
                                                 .getY();
      {
        boost::lock_guard<boost::recursive_mutex> lock(reconfigure_mutex_);
        reconfigure_server_.updateConfig(config);
      }
    }

    void reconfigureParameters(tsuten_behavior::TableManagerConfig &config, uint32_t level)
    {
      table_tfs_.at(TableID::MOVABLE_TABLE_1200)
          .getOrigin()
          .setY(config.movable_table_1200_position_y);
      table_tfs_.at(TableID::MOVABLE_TABLE_1500)
          .getOrigin()
          .setY(config.movable_table_1500_position_y);
      table_tfs_.at(TableID::MOVABLE_TABLE_1800)
          .getOrigin()
          .setY(config.movable_table_1800_position_y);
    }

    ros::NodeHandle pnh_;

    ros::Timer publish_tf_timer_;

    boost::recursive_mutex reconfigure_mutex_;

    dynamic_reconfigure::Server<tsuten_behavior::TableManagerConfig> reconfigure_server_;

    tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;

    std::unordered_map<TableID, tf2::Transform> table_tfs_;

    std::string global_frame_;

    double tf_publish_rate_;
  };

  const std::unordered_map<TableID, tf2::Transform> TableManager::DEFAULT_TABLE_TFS =
      {{TableID::DUAL_TABLE, tf2::Transform(tf2::Quaternion::getIdentity(),
                                            {2.5, 2.4, 0})},
       {TableID::MOVABLE_TABLE_1200, tf2::Transform(tf2::Quaternion::getIdentity(),
                                                    {4.5, 1.9, 0})},
       {TableID::MOVABLE_TABLE_1500, tf2::Transform(tf2::Quaternion::getIdentity(),
                                                    {4.5, 1.9, 0})},
       {TableID::MOVABLE_TABLE_1800, tf2::Transform(tf2::Quaternion::getIdentity(),
                                                    {6.5, 1.9, 0})}};
} // namespace tsuten_behavior

int main(int argc, char **argv)
{
  ros::init(argc, argv, "table_manager");

  tsuten_behavior::TableManager table_manager;

  ros::spin();

  return 0;
}