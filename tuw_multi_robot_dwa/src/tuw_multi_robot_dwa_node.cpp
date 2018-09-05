//
// Created by Bassel Mahmoud on 01.08.18.
//

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>

#include <tuw_multi_robot_dwa_node.h>

int main(int argc, char** argv)
{
    if (argc >= 2)
    {
        ros::init(argc, argv, argv[1]);

    } else {
        ROS_INFO("Please specifie name \nrosrun tuw_multi_robot_dwa tuw_multi_robot_dwa_node [name]");
    }
}