#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include "multi_agent_planner/cf_manager.h"
#include "multi_agent_planner/obstacle.h"

using namespace ghostplanner::cfplanner;

int main(int argc, char** argv) {
    ros::init(argc, argv, "cf_agent_demo");
    ros::NodeHandle nh;

    ROS_INFO("Starting Circular Field Agent Demo...");

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher goal_pub = nh.advertise<geometry_msgs::Point>("goal_position", 10);
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float64>("distance_to_goal", 10);
    tf2_ros::TransformBroadcaster tf_broadcaster;

    Eigen::Vector3d start_pos(0.0, 0.0, 0.0);
    Eigen::Vector3d goal_pos(10.0, 10.0, 0.0);

    ROS_INFO("Start position: [%.2f, %.2f, %.2f]", start_pos.x(), start_pos.y(), start_pos.z());
    ROS_INFO("Goal position: [%.2f, %.2f, %.2f]", goal_pos.x(), goal_pos.y(), goal_pos.z());

    std::vector<Obstacle> obstacles = {
        Obstacle("Obstacle1", Eigen::Vector3d(5.0, 5.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), 0.8),
        Obstacle("Obstacle2", Eigen::Vector3d(6.0, 7.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), 0.8),
        Obstacle("Obstacle3", Eigen::Vector3d(8.0, 6.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), 0.6),
        Obstacle("Obstacle4", Eigen::Vector3d(2.0, 3.0, 0.0), Eigen::Vector3d(0.0, 0.0, 0.0), 0.6)
    };

    for (const auto& obs : obstacles) {
        ROS_INFO("Obstacle %s created at position: [%.2f, %.2f, %.2f] with radius %.2f",
                 obs.getName().c_str(), obs.getPosition().x(), obs.getPosition().y(), obs.getPosition().z(), obs.getRadius());
    }

    double detect_shell_rad = 0.35;
    double agent_mass = 1.0;
    double agent_radius = 0.5;
    double velocity_max = 1.0;
    std::vector<double> k_a_ee{4.0, 4.0, 4.0, 4.0};
    std::vector<double> k_c_ee{2.0, 4.0, 4.0, 4.0};
    std::vector<double> k_r_ee{2.0, 4.0, 4.0, 4.0};
    std::vector<double> k_r_force{0.0, 0.0, 0.0, 0.0};
    std::vector<double> k_d_ee{1.0, 1.0, 1.0, 1.0};
    std::vector<double> k_manip{0.0, 0.0, 0.0, 0.0};

    CfManager cf_manager(start_pos, goal_pos, 0.1, obstacles, k_a_ee, k_c_ee, k_r_ee, k_d_ee, k_r_force, k_manip);

    ros::Rate rate(10);
    bool planning_active = true;
    bool open_loop = false;

    // 可视化起点和终点
    visualization_msgs::Marker start_marker, goal_marker;
    start_marker.header.frame_id = goal_marker.header.frame_id = "map";
    start_marker.ns = goal_marker.ns = "cf_agent_demo";
    start_marker.id = 0;
    goal_marker.id = 1;
    start_marker.type = goal_marker.type = visualization_msgs::Marker::SPHERE;
    start_marker.action = goal_marker.action = visualization_msgs::Marker::ADD;
    start_marker.scale.x = start_marker.scale.y = start_marker.scale.z = 0.5;
    goal_marker.scale.x = goal_marker.scale.y = goal_marker.scale.z = 0.5;
    start_marker.color.r = 0.0;
    start_marker.color.g = 1.0;
    start_marker.color.b = 0.0;
    start_marker.color.a = 1.0;
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 0.0;
    goal_marker.color.a = 1.0;
    start_marker.pose.position.x = start_pos.x();
    start_marker.pose.position.y = start_pos.y();
    start_marker.pose.position.z = start_pos.z();
    goal_marker.pose.position.x = goal_pos.x();
    goal_marker.pose.position.y = goal_pos.y();
    goal_marker.pose.position.z = goal_pos.z();

    // 可视化障碍物
    std::vector<visualization_msgs::Marker> obstacle_markers;
    for (size_t i = 0; i < obstacles.size(); ++i) {
        visualization_msgs::Marker obs_marker;
        obs_marker.header.frame_id = "map";
        obs_marker.ns = "cf_agent_demo_obstacles";
        obs_marker.id = static_cast<int>(i + 10);
        obs_marker.type = visualization_msgs::Marker::SPHERE;
        obs_marker.action = visualization_msgs::Marker::ADD;
        obs_marker.scale.x = obs_marker.scale.y = obs_marker.scale.z = obstacles[i].getRadius() * 2.0;
        obs_marker.color.r = 0.0;
        obs_marker.color.g = 0.0;
        obs_marker.color.b = 1.0;
        obs_marker.color.a = 1.0;
        obs_marker.pose.position.x = obstacles[i].getPosition().x();
        obs_marker.pose.position.y = obstacles[i].getPosition().y();
        obs_marker.pose.position.z = obstacles[i].getPosition().z();
        obstacle_markers.push_back(obs_marker);
    }

    // 可视化智能体当前位置
    visualization_msgs::Marker agent_marker;
    agent_marker.header.frame_id = "map";
    agent_marker.ns = "cf_agent_demo_agents";
    agent_marker.id = 100;
    agent_marker.type = visualization_msgs::Marker::SPHERE;
    agent_marker.action = visualization_msgs::Marker::ADD;
    agent_marker.scale.x = agent_marker.scale.y = agent_marker.scale.z = 0.5;
    agent_marker.color.r = 1.0;
    agent_marker.color.g = 1.0;
    agent_marker.color.b = 0.0;
    agent_marker.color.a = 1.0;

    // 可视化智能体的真实轨迹
    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.header.frame_id = "map";
    trajectory_marker.ns = "cf_agent_demo_trajectory";
    trajectory_marker.id = 101;
    trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::Marker::ADD;
    trajectory_marker.scale.x = 0.05;
    trajectory_marker.color.r = 1.0;
    trajectory_marker.color.g = 1.0;
    trajectory_marker.color.b = 0.0;
    trajectory_marker.color.a = 1.0;

    while (ros::ok()) {
        if (planning_active) {
            double start_plan_timestamp = ros::Time::now().toSec();

            // 发布起点和终点
            marker_pub.publish(start_marker);
            marker_pub.publish(goal_marker);

            // 发布障碍物
            for (const auto& obs_marker : obstacle_markers) {
                marker_pub.publish(obs_marker);
            }

            // 更新真实执行智能体位置
            if (!open_loop) {
                cf_manager.setRealEEAgentPosition(start_pos);
                open_loop = true;
            }

            // 停止路径预测
            cf_manager.stopPrediction();

            // 评估智能体并选择最佳路径
            int best_agent_id = cf_manager.evaluateAgents(obstacles, 1.0, 1.0, 1.0, 1.0, Eigen::Matrix<double, 6, 1>::Zero());
            ROS_INFO("Best agent ID: %d", best_agent_id);

            // 可视化所有预测路径
            const auto& predicted_paths = cf_manager.getPredictedPaths();
            for (size_t i = 0; i < predicted_paths.size(); ++i) {
                const auto& path = predicted_paths[i];
                if (path.size() > 2) {
                    visualization_msgs::Marker path_marker;
                    path_marker.header.frame_id = "map";
                    path_marker.ns = "cf_agent_demo_paths";
                    path_marker.id = static_cast<int>(i + 20);
                    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
                    path_marker.action = visualization_msgs::Marker::ADD;
                    path_marker.scale.x = 0.05;
                    path_marker.color.r = (i == best_agent_id) ? 1.0 : 0.0;
                    path_marker.color.g = 0.0;
                    path_marker.color.b = (i == best_agent_id) ? 0.0 : 1.0;
                    path_marker.color.a = 1.0;

                    for (const Eigen::Vector3d& point : path) {
                        geometry_msgs::Point ros_point;
                        ros_point.x = point.x();
                        ros_point.y = point.y();
                        ros_point.z = point.z();
                        path_marker.points.push_back(ros_point);
                    }

                    marker_pub.publish(path_marker);
                }
            }

            Eigen::Vector3d current_agent_pos = cf_manager.getNextPosition();
            ROS_INFO("Current agent position: [%.2f, %.2f, %.2f]", current_agent_pos.x(), current_agent_pos.y(), current_agent_pos.z());

            // 可视化智能体当前位置
            agent_marker.pose.position.x = current_agent_pos.x();
            agent_marker.pose.position.y = current_agent_pos.y();
            agent_marker.pose.position.z = current_agent_pos.z();
            marker_pub.publish(agent_marker);

            // 更新真实轨迹
            geometry_msgs::Point trajectory_point;
            trajectory_point.x = current_agent_pos.x();
            trajectory_point.y = current_agent_pos.y();
            trajectory_point.z = current_agent_pos.z();
            trajectory_marker.points.push_back(trajectory_point);
            marker_pub.publish(trajectory_marker);

            // 移动最佳路径的智能体
            cf_manager.moveRealEEAgent(obstacles, 0.1, 1, best_agent_id);

            Eigen::Vector3d updated_position = cf_manager.getNextPosition();
            ROS_INFO("After moveRealEEAgent, position: [%.2f, %.2f, %.2f]", updated_position.x(), updated_position.y(), updated_position.z());

            double end_plan_timestamp = ros::Time::now().toSec();
            ROS_INFO("Planning time: %.3f seconds", end_plan_timestamp - start_plan_timestamp);

            // 重置智能体并启动下一轮预测
            cf_manager.resetEEAgents(updated_position, cf_manager.getNextVelocity(), obstacles);
            cf_manager.startPrediction();

            // 发布目标位置
            geometry_msgs::Point goal_msg;
            goal_msg.x = cf_manager.getNextPosition().x();
            goal_msg.y = cf_manager.getNextPosition().y();
            goal_msg.z = cf_manager.getNextPosition().z();
            goal_pub.publish(goal_msg);

            // 发布目标距离
            std_msgs::Float64 dist_msg;
            dist_msg.data = cf_manager.getDistFromGoal();
            dist_pub.publish(dist_msg);
        } else {
            ROS_INFO_STREAM("Planning not active. Setting initial position.");
            cf_manager.setInitialPosition(start_pos);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
