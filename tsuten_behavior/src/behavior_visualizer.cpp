#include <interactive_markers/interactive_marker_server.h>

#include <tsuten_behavior/behavior_server.hpp>
#include <tsuten_behavior/constants.hpp>

namespace tsuten_behavior
{
  class BehaviorVisualizer
  {
  public:
    BehaviorVisualizer() : table_markers_server_("table_markers")
    {
      ros::NodeHandle pnh("~");
    }

  private:
    interactive_markers::InteractiveMarkerServer table_markers_server_;
  };
} // namespace tsuten_behavior

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_visualizer");

  tsuten_behavior::BehaviorVisualizer behavior_visualizer;

  ros::spin();

  return 0;
}