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

        ros::NodeHandle nh;

        //TODO: check if rate is enough
        ros::Rate r(20);

        //init DWAPlannerROS
        tf::TransformListener tf(ros::Duration(10));

        // TODO: subscribe to the global planner
        //TODO: add name of the cost map every robot will need a cost map
        costmap_2d::Costmap2DROS costmap("my_costmap", tf);

        dwa_local_planner::DWAPlannerROS dp;

        //TODO: add unique name for the dwa since this will be used in multirobot environment
        dp.initialize("my_dwa_planner", &tf, &costmap);

        while(ros::ok())
        {
            ros::spinOnce();
            r.sleep();
        }

        return 0;
    } else {
        ROS_INFO("Please specifie name \nrosrun tuw_multi_robot_dwa tuw_multi_robot_dwa_node [name]");
        return 1;
    }


}