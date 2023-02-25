#include <ros/ros.h>

#include <tsuten_mechanism/bottle_shooter_controller.hpp>
#include <tsuten_msgs/ResetShooter.h>
#include <tsuten_msgs/ShootBottle.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bottle_shooter_controller_node");

    ros::NodeHandle pnh("~");

    std::string bottle_shooter_name =
        pnh.param("bottle_shooter_name", std::string("bottle_shooter"));
    double valve_on_duration = pnh.param("valve_on_duration", 0.5);

    tsuten_mechanism::BottleShooterController bottle_shooter_controller(bottle_shooter_name, valve_on_duration);

    ros::ServiceServer reset_shooter_service_server =
        pnh.advertiseService<tsuten_msgs::ResetShooterRequest, tsuten_msgs::ResetShooterResponse>(
            "reset_shooter",
            [&](auto &, auto &)
            { bottle_shooter_controller.resetShooter(); return true; });

    ros::ServiceServer shoot_bottle_service_server =
        pnh.advertiseService<tsuten_msgs::ShootBottleRequest, tsuten_msgs::ShootBottleResponse>(
            "shoot_bottle",
            [&](auto &, auto &)
            { bottle_shooter_controller.shootBottle(); return true; });

    ros::spin();

    return 0;
}