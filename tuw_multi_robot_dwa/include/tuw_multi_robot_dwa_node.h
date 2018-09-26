//
// Created by Bassel Mahmoud on 01.08.18.
//
#ifndef TUW_MULTI_ROBOT_DWA_NODE_H
#define TUW_MULTI_ROBOT_DWA_NODE_H

// ROS
#include <ros/ros.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>

#include <tuw_multi_robot_msgs/Route.h>
#include <tuw_multi_robot_msgs/RobotInfo.h>

#include <memory>

namespace dwa_controller
{
class LocalDwaMultiRobotControllerNode
{
    //
public:
    // TODO: add documenatation
    LocalDwaMultiRobotControllerNode(ros::NodeHandle &n);
    ros::NodeHandle n_;
    ros::NodeHandle n_param_;
    std::unique_ptr<ros::Rate> rate_;

    void publishRobotInfo();

private:
    std::vector<std::string> robots_names_;
    std::vector<float> robots_radius_;
    std::vector<geometry_msgs::PoseWithCovariance> robots_poses_;

    std::vector<dwa_local_planner::DWAPlannerROS> dwaPlanners_;

    std::vector<costmap_2d::Costmap2DROS> costMaps_;
    ros::Publisher pubRobotInfo_;

    // subscribers
    std::vector<ros::Subscriber> subOdom_;
    std::vector<ros::Subscriber> subRoute_;
    std::vector<ros::Subscriber> subCtrl_;

    // publishers
    std::vector<ros::Publisher> pubCmdVel_;

    // callbacks
    void subOdomCb(const ros::MessageEvent<nav_msgs::Odometry const > &_event, int _topic);
    void subRouteCb(const ros::MessageEvent<tuw_multi_robot_msgs::Route const > &_event, int _topic);
    void subCtrlCb(const ros::MessageEvent<std_msgs::String const > &_event, int _topic);

    // Helpers
    int findRobotId(const std::string& robot_name);
};
} // namespace dwa_controller



#endif //TUW_MULTI_ROBOT_DWA_NODE_H
