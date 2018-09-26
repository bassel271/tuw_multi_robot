//
// Created by Bassel Mahmoud on 01.08.18.
//

#include "ros/ros.h"
#include <tf/transform_listener.h>
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

namespace dwa_controller
{
LocalDwaMultiRobotControllerNode::LocalDwaMultiRobotControllerNode(ros::NodeHandle &n) : n_(n),
                                                                                    n_param_("~"),
                                                                                    robots_names_(std::vector<std::string>({"robot0"}))
{
    // TODO
}

void dwa_controller::LocalDwaMultiRobotControllerNode::subOdomCb(
        const ros::MessageEvent<nav_msgs::Odometry const> &_event, int _topic)
{
    // TODO
}

void dwa_controller::LocalDwaMultiRobotControllerNode::subRouteCb(
        const ros::MessageEvent<tuw_multi_robot_msgs::Route const> &_event, int _topic)
{
    // TODO
}

void dwa_controller::LocalDwaMultiRobotControllerNode::subCtrlCb(
        const ros::MessageEvent<std_msgs::String const> &_event, int _topic) {
    // TODO
}

int dwa_controller::LocalDwaMultiRobotControllerNode::findRobotId(const std::string& robot_name)
{
    auto it = std::find(robots_names_.begin(), robots_names_.end(), robot_name);

    if (it != robots_names_.end())
    {
        return std::distance(robots_names_.begin(), it);
    }
    return -1;
}

void dwa_controller::LocalDwaMultiRobotControllerNode::publishRobotInfo() {
    //TODO
}

} // namespace dwa_controller