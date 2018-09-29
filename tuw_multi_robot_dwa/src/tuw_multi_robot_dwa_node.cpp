//
// Created by Bassel Mahmoud on 01.08.18.
//

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <tuw_multi_robot_dwa_node.h>

#include <algorithm>

int main(int argc, char** argv)
{
    if (argc >= 2)
    {
        ros::init(argc, argv, argv[1]);

        ros::NodeHandle nh;

        dwa_controller::LocalDwaMultiRobotControllerNode ctrl(nh);

//        //TODO: check if rate is enough
//        ros::Rate r(20);
//
//        //init DWAPlannerROS
//        tf::TransformListener tf(ros::Duration(10));
//
//        // TODO: subscribe to the global planner
//        //TODO: add name of the cost map every robot will need a cost map
//        costmap_2d::Costmap2DROS costmap("my_costmap", tf);
//
//        dwa_local_planner::DWAPlannerROS dp;
//
//        //TODO: add unique name for the dwa since this will be used in multirobot environment
//        dp.initialize("my_dwa_planner", &tf, &costmap);
//
        ros::Rate r(ctrl.getUpdateRate());
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
                                                                                    robots_names_(std::vector<std::string>({"robot0"})) {
    n_param_.param("nr_of_robots", nr_of_robots_, 0);
    n_param_.param<std::string>("robot_prefix", robot_prefix_, "robot_");
    std::string robot_names_string = "";
    n_param_.param("robot_names_str", robot_names_string, robot_names_string);

    if ((nr_of_robots_ == 0) && robot_names_string.empty()) {
        ROS_ERROR("One of the parameters nr_of_robots or robot_names_str should be defind");
    }
    if ((nr_of_robots_ > 0) && !robot_names_string.empty()) {
        ROS_ERROR(
                "one of the parameters nr_of_robots or robot_names_string need to be defined, nr_of_robots will be ignored");
    }
    if (robot_names_string.size() > 0) {
        robot_names_string.erase(std::remove_if(robot_names_string.begin(), robot_names_string.end(), isspace),
                                 robot_names_string.end());
        std::istringstream stringStr(robot_names_string);
        std::string result;
        robots_names_.clear();

        while (std::getline(stringStr, result, ',')) {
            robots_names_.push_back(result);
        }

        nr_of_robots_ = robots_names_.size();

    } else {
        robots_names_.resize(nr_of_robots_);
        for (int i = 0; i < nr_of_robots_; i++) {
            robots_names_[i] = robot_prefix_ + std::to_string(i);
        }
    }

    dwaPlanners_.resize(robots_names_.size());
    robots_poses_.resize(robots_names_.size());
    costMaps_.resize(robots_names_.size());

    subOdom_.resize(robots_names_.size());
    subRoute_.resize(robots_names_.size());
    subCtrl_.resize(robots_names_.size());
    pubCmdVel_.resize(robots_names_.size());

    //Robot radius can also be set as string and as array in yaml file
    float default_radius = 0.3;
    n_param_.param("robot_default_radius", default_radius, default_radius);
    robots_radius_.resize(robots_names_.size(), default_radius);

    topic_odom_ = "odom";
    n.getParam("odom_topic", topic_odom_);

    topic_cmdVel_ = "cmd_vel";
    n.getParam("cmd_vel_topic", topic_cmdVel_);

    topic_route_ = "route";
    n.getParam("route_topic", topic_route_);

    topic_robot_info_ = "/robot_info";
    n.getParam("robot_info_topic", topic_robot_info_);

    max_vel_v_ = 0.8;
    n.getParam("max_v", max_vel_v_);

    max_vel_w_ = 1.0;
    n.getParam("max_w", max_vel_w_);

    goal_r_ = 0.2;
    n.getParam("goal_radius", goal_r_);

    topic_ctrl_ = "/ctrl";
    n.getParam("topic_control", topic_ctrl_);

    n_param_.param<double>("update_rate", update_rate_, 20.0);

    n_param_.param<double>("update_rate_info", update_rate_info_, 1.0);

    // TODO: initialise dwa planners and cost maps for all the robots

    for (auto &planner : dwaPlanners_) {
        // TODO: set dwa planner parameters
    }

    for (int i = 0; i < robots_names_.size(); i++) {
        pubCmdVel_[i] = n.advertise<geometry_msgs::Twist>(robots_names_[i] + "/" + topic_cmdVel_, 1);
        pubRobotInfo_ = n.advertise<tuw_multi_robot_msgs::RobotInfo>(topic_robot_info_, robots_names_.size() * 2);
        subOdom_[i] = n.subscribe<nav_msgs::Odometry>(robots_names_[i] + "/" + topic_odom_, 1,
                                                      boost::bind(&LocalDwaMultiRobotControllerNode::subOdomCb, this,
                                                                  _1, i));
        subRoute_[i] = n.subscribe<tuw_multi_robot_msgs::Route>(robots_names_[i] + "/" + topic_route_, 1, boost::bind(
                &LocalDwaMultiRobotControllerNode::subRouteCb, this, _1, i));
        subCtrl_[i] = n.subscribe<std_msgs::String>(robots_names_[i] + "/" + topic_ctrl_, 1,
                                                    boost::bind(&LocalDwaMultiRobotControllerNode::subCtrlCb, this, _1,
                                                                i));
    }
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