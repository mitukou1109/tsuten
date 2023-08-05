#include <ros/ros.h>
#include <ros/package.h>
#include <jsk_rviz_plugins/OverlayText.h>
#include <std_msgs/Bool.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include <tsuten_behavior/constants.hpp>
#include <tsuten_mechanism/solenoid_valve_controller.hpp>
#include <tsuten_msgs/PerformAction.h>

namespace tsuten_behavior
{
  class BehaviorVisualizer
  {
  public:
    BehaviorVisualizer() : tf_listener_(tf_buffer_)
    {
      ros::NodeHandle nh, pnh("~"), behavior_server_nh("behavior_server");

      initializeShooterStates();

      for (auto &shooter_name_pair : SHOOTER_NAMES)
      {
        auto &shooter_id = shooter_name_pair.first;
        auto &shooter_name = shooter_name_pair.second;

        auto shooter_state_sub = nh.subscribe<std_msgs::Bool>(
            shooter_name + "/command", 2,
            boost::bind(&BehaviorVisualizer::setShooterState, this, shooter_id, _1));

        shooter_state_subs_.insert({shooter_id, shooter_state_sub});
      }

      pnh.param("global_frame", global_frame_, std::string("map"));
      pnh.param("table_markers_publish_rate", table_markers_publish_rate_, 1.0);

      shooter_state_text_pub_ =
          pnh.advertise<jsk_rviz_plugins::OverlayText>("shooter_state_text", 10, true);

      table_markers_pub_ =
          pnh.advertise<visualization_msgs::MarkerArray>("table_markers", 10);

      displayShooterStates();

      createTableMarkers();

      publish_table_markers_timer_ =
          pnh.createTimer(ros::Rate(table_markers_publish_rate_),
                          &BehaviorVisualizer::publishTableMarkers, this);

      initializePerformFeedbackText();

      perform_feedback_text_pub_ =
          pnh.advertise<jsk_rviz_plugins::OverlayText>("perform_feedback_text", 10, true);

      perform_feedback_sub_ =
          behavior_server_nh.subscribe("perform/feedback", 10,
                                       &BehaviorVisualizer::displayPerformFeedback, this);

      perform_result_sub_ =
          behavior_server_nh.subscribe("perform/result", 10,
                                       &BehaviorVisualizer::hidePerformFeedback, this);
    }

  private:
    using ValveState = tsuten_mechanism::SolenoidValveController::State;

    static const std::unordered_map<ValveState, std::string> SHOOTER_STATE_TEXTS;

    static const std::unordered_map<TableBaseID, std::array<float, 4>>
        TABLE_MARKER_COLORS;

    void initializeShooterStates()
    {
      for (auto &shooter_name_pair : SHOOTER_NAMES)
      {
        auto &shooter_id = shooter_name_pair.first;

        shooter_states_.emplace(shooter_id, ValveState::OFF);
      }

      shooter_state_text_.action = jsk_rviz_plugins::OverlayText::ADD;
      shooter_state_text_.width = 280;
      shooter_state_text_.height = 120;
      shooter_state_text_.left = 10;
      shooter_state_text_.top = 10;
      shooter_state_text_.bg_color.r = 0;
      shooter_state_text_.bg_color.g = 0;
      shooter_state_text_.bg_color.b = 0;
      shooter_state_text_.bg_color.a = 0.4;
      shooter_state_text_.fg_color.r = 0.1;
      shooter_state_text_.fg_color.g = 1.0;
      shooter_state_text_.fg_color.b = 1.0;
      shooter_state_text_.fg_color.a = 0.8;
      shooter_state_text_.line_width = 1;
      shooter_state_text_.text_size = 12;
      shooter_state_text_.font = "Ubuntu";
    }

    void displayShooterStates()
    {
      std::string shooter_state_text;
      for (auto &shooter_state_pair : shooter_states_)
      {
        auto &shooter_id = shooter_state_pair.first;
        auto &shooter_state = shooter_state_pair.second;

        shooter_state_text +=
            SHOOTER_TEXTS.at(shooter_id) + ": " + SHOOTER_STATE_TEXTS.at(shooter_state) + "\n";
      }

      shooter_state_text_.text = shooter_state_text;

      shooter_state_text_pub_.publish(shooter_state_text_);
    }

    void createTableMarkers()
    {
      const std::string table_mesh_resource_dir =
          ros::package::getPath("tsuten_description") + "/meshes";

      for (auto &table_name_pair : TABLE_BASE_NAMES)
      {
        auto &table_base_id = table_name_pair.first;
        auto table_base_name = table_name_pair.second;

        visualization_msgs::Marker table_marker;
        table_marker.header.frame_id = global_frame_;
        table_marker.ns = table_base_name;
        table_marker.id = static_cast<int32_t>(table_base_id);
        table_marker.type = visualization_msgs::Marker::CUBE;
        // table_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        // table_marker.mesh_resource = table_mesh_resource_dir + "/" + table_name + ".stl";
        table_marker.scale.x = TABLE_SIZES.at(table_base_id).at(0);
        table_marker.scale.y = TABLE_SIZES.at(table_base_id).at(1);
        table_marker.scale.z = TABLE_SIZES.at(table_base_id).at(2);
        table_marker.color.r = TABLE_MARKER_COLORS.at(table_base_id).at(0);
        table_marker.color.g = TABLE_MARKER_COLORS.at(table_base_id).at(1);
        table_marker.color.b = TABLE_MARKER_COLORS.at(table_base_id).at(2);
        table_marker.color.a = TABLE_MARKER_COLORS.at(table_base_id).at(3);

        table_markers_.insert({table_base_id, table_marker});
      }
    }

    void setShooterState(const ShooterID &shooter_id, const std_msgs::Bool::ConstPtr &msg)
    {
      shooter_states_.at(shooter_id) = static_cast<ValveState>(msg->data);

      displayShooterStates();
    }

    void publishTableMarkers(const ros::TimerEvent &event)
    {
      visualization_msgs::MarkerArray table_markers_msg;

      for (auto &table_marker_pair : table_markers_)
      {
        auto &table_base_id = table_marker_pair.first;
        auto &table_marker = table_marker_pair.second;

        table_marker.header.stamp = ros::Time::now();

        tf2::Transform tf;
        if (getTableTF(table_base_id, tf))
        {
          tf2::toMsg(
              tf * tf2::Transform(tf2::Quaternion::getIdentity(),
                                  {0, 0, TABLE_SIZES.at(table_base_id).at(2) / 2}),
              table_marker.pose);
          table_markers_msg.markers.push_back(table_marker);
        }
      }

      table_markers_pub_.publish(table_markers_msg);
    }

    bool getTableTF(TableBaseID table_id, tf2::Transform &tf)
    {
      try
      {
        tf2::fromMsg(
            tf_buffer_.lookupTransform(global_frame_, TABLE_BASE_NAMES.at(table_id) + "_link",
                                       ros::Time(0), ros::Duration(1.0))
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

    void initializePerformFeedbackText()
    {
      perform_feedback_text_.width = 380;
      perform_feedback_text_.height = 20;
      perform_feedback_text_.left = 10;
      perform_feedback_text_.top = 140;
      perform_feedback_text_.bg_color.r = 0;
      perform_feedback_text_.bg_color.g = 0;
      perform_feedback_text_.bg_color.b = 0;
      perform_feedback_text_.bg_color.a = 0.4;
      perform_feedback_text_.fg_color.r = 0.1;
      perform_feedback_text_.fg_color.g = 1.0;
      perform_feedback_text_.fg_color.b = 1.0;
      perform_feedback_text_.fg_color.a = 0.8;
      perform_feedback_text_.line_width = 1;
      perform_feedback_text_.text_size = 10;
      perform_feedback_text_.font = "Ubuntu";
    }

    void displayPerformFeedback(const tsuten_msgs::PerformActionFeedback &feedback)
    {
      perform_feedback_text_.action = jsk_rviz_plugins::OverlayText::ADD;
      perform_feedback_text_.text =
          "Perform status: " + PERFORM_STATUS_TEXTS.at(feedback.feedback.status);

      perform_feedback_text_pub_.publish(perform_feedback_text_);
    }

    void hidePerformFeedback(const tsuten_msgs::PerformActionResult &result)
    {
      perform_feedback_text_.action = jsk_rviz_plugins::OverlayText::DELETE;

      perform_feedback_text_pub_.publish(perform_feedback_text_);
    }

    ros::Publisher shooter_state_text_pub_;

    std::unordered_map<ShooterID, ros::Subscriber> shooter_state_subs_;

    std::unordered_map<ShooterID, ValveState> shooter_states_;

    jsk_rviz_plugins::OverlayText shooter_state_text_;

    ros::Publisher table_markers_pub_;

    ros::Timer publish_table_markers_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::unordered_map<TableBaseID, visualization_msgs::Marker> table_markers_;

    ros::Publisher perform_feedback_text_pub_;

    ros::Subscriber perform_feedback_sub_;
    ros::Subscriber perform_result_sub_;

    jsk_rviz_plugins::OverlayText perform_feedback_text_;

    std::string global_frame_;

    double table_markers_publish_rate_;
  };

  const std::unordered_map<BehaviorVisualizer::ValveState, std::string>
      BehaviorVisualizer::SHOOTER_STATE_TEXTS =
          {{ValveState::OFF, "<span style=\"color : blue;\">OFF</span>"},
           {ValveState::ON, "<span style=\"color : red;\">ON</span> "}};

  const std::unordered_map<TableBaseID, std::array<float, 4>>
      BehaviorVisualizer::TABLE_MARKER_COLORS =
          {{TableBaseID::DUAL_TABLE, {1.0, 0.0, 0.0, 1.0}},
           {TableBaseID::MOVABLE_TABLE_1200, {1.0, 1.0, 0.0, 1.0}},
           {TableBaseID::MOVABLE_TABLE_1500, {1.0, 1.0, 0.0, 1.0}},
           {TableBaseID::MOVABLE_TABLE_1800, {1.0, 1.0, 0.0, 1.0}}};
} // namespace tsuten_behavior

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_visualizer");

  tsuten_behavior::BehaviorVisualizer behavior_visualizer;

  ros::spin();

  return 0;
}