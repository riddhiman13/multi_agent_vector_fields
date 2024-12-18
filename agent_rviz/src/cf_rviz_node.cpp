#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "bimanual_planning_ros/cf_manager.h"

using ghostplanner::cfplanner::Obstacle;
using ghostplanner::cfplanner::CfManager;

int main(int argc, char **argv) {
    ros::init(argc, argv, "cf_rviz_node");
    ros::NodeHandle nh;

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

    Eigen::Vector3d start_pos(1.0, 1.0, 0.0); 
    Eigen::Vector3d goal_pos(7.0, 7.0, 0.0);   

    // 障碍物初始化
    std::vector<Obstacle> obstacles = {
        Obstacle(Eigen::Vector3d(2.0, 2.0, 0.0), 0.4),
        Obstacle(Eigen::Vector3d(4.0, 4.0, 0.0), 0.3),
        Obstacle(Eigen::Vector3d(5.0, 5.0, -5.0), 0.2)
    };

    // 初始化 CfManager
    ROS_INFO("CfManager...");
    CfManager manager(start_pos, goal_pos, 0.1, obstacles,
                      {3.0, 3.0, 4.0},  // k_a_ee
                      {1.5, 2.5, 3.5},  // k_c_ee
                      {1.0, 0.7, 0.9},  // k_r_ee
                      {1.1, 0.2, 0.3},  // k_d_ee
                      {1.0, 1.0, 1.0},  // k_manip
                      {1.5, 0.6, 0.7},  // k_r_force
                      1.0, 0.1, 0.5);   
    ROS_INFO("CfManager successfully.");

    ros::Rate rate(10);

    visualization_msgs::Marker start_marker, goal_marker, agent_marker, obstacle_marker, path_marker;

    // Start
    start_marker.header.frame_id = "map";
    start_marker.ns = "start";
    start_marker.id = 0;
    start_marker.type = visualization_msgs::Marker::SPHERE;
    start_marker.action = visualization_msgs::Marker::ADD;
    start_marker.pose.position.x = start_pos.x();
    start_marker.pose.position.y = start_pos.y();
    start_marker.pose.position.z = start_pos.z();
    start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.5;
    start_marker.color.r = 1.0;
    start_marker.color.g = 1.0;
    start_marker.color.b = 0.0;
    start_marker.color.a = 1.0;

    // Goal
    goal_marker.header.frame_id = "map";
    goal_marker.ns = "goal";
    goal_marker.id = 1;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.position.x = goal_pos.x();
    goal_marker.pose.position.y = goal_pos.y();
    goal_marker.pose.position.z = goal_pos.z();
    goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.5;
    goal_marker.color.g = 1.0;
    goal_marker.color.a = 1.0;

    // Agent
    agent_marker.header.frame_id = "map";
    agent_marker.ns = "agent";
    agent_marker.id = 2;
    agent_marker.type = visualization_msgs::Marker::SPHERE;
    agent_marker.action = visualization_msgs::Marker::ADD;
    agent_marker.scale.x = agent_marker.scale.y = agent_marker.scale.z = 0.3;
    agent_marker.color.r = 1.0;
    agent_marker.color.a = 1.0;

    // Obstacle 
    obstacle_marker.header.frame_id = "map";
    obstacle_marker.ns = "obstacles";
    obstacle_marker.type = visualization_msgs::Marker::SPHERE;
    obstacle_marker.action = visualization_msgs::Marker::ADD;
    obstacle_marker.color.b = 1.0;
    obstacle_marker.color.a = 1.0;

    // Path
    path_marker.header.frame_id = "map";
    path_marker.ns = "path";
    path_marker.id = 3;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.05;
    path_marker.color.r = 0.5;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.5;
    path_marker.color.a = 1.0;

    while (ros::ok()) {
        ROS_INFO("planning step...");
       
        manager.moveRealEEAgent(obstacles, 0.1, 1, 0);  //  RealCfAgent (

        Eigen::Vector3d pos = manager.getRealEEAgentPosition();
        agent_marker.pose.position.x = pos.x();
        agent_marker.pose.position.y = pos.y();
        agent_marker.pose.position.z = pos.z();

        geometry_msgs::Point path_point;
        path_point.x = pos.x();
        path_point.y = pos.y();
        path_point.z = pos.z();
        path_marker.points.push_back(path_point);

        // pub marker   
        marker_pub.publish(start_marker);
        marker_pub.publish(goal_marker);

        for (size_t i = 0; i < obstacles.size(); ++i) {
            obstacle_marker.id = 4 + i;
            obstacle_marker.pose.position.x = obstacles[i].getPosition().x();
            obstacle_marker.pose.position.y = obstacles[i].getPosition().y();
            obstacle_marker.pose.position.z = obstacles[i].getPosition().z();
            obstacle_marker.scale.x = obstacle_marker.scale.y = obstacle_marker.scale.z = obstacles[i].getRadius() * 2.0;
            marker_pub.publish(obstacle_marker);
        }

        // pub agent / path 
        marker_pub.publish(agent_marker);
        marker_pub.publish(path_marker);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
