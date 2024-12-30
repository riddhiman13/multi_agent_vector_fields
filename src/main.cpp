#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include "multi_agent_vector_fields/cf_manager.h"
#include "multi_agent_vector_fields/obstacle.h"

using namespace ghostplanner::cfplanner;

void visualizeMarker(ros::Publisher& marker_pub, const Eigen::Vector3d& position, int id, const std::string& ns,
                     const std::string& frame_id, double scale, double r, double g, double b, double a, int type = visualization_msgs::Marker::SPHERE) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = scale;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker_pub.publish(marker);
}

void publishAgentFrame(tf2_ros::TransformBroadcaster& tf_broadcaster, const Eigen::Vector3d& position) {
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "agent_frame";

    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation.x = 0.0;
    transform.transform.rotation.y = 0.0;
    transform.transform.rotation.z = 0.0;
    transform.transform.rotation.w = 1.0;

    tf_broadcaster.sendTransform(transform);
}

Eigen::Vector3d readVector3d(const YAML::Node& node) 
{
    auto vec = node.as<std::vector<double>>();
    return Eigen::Vector3d(vec[0], vec[1], vec[2]);
}

std::vector<Obstacle> readObstacles(const YAML::Node& node) 
{
    std::vector<Obstacle> obstacles;
    for (const auto& obstacle : node) {
        std::string name = obstacle["name"].as<std::string>();
        Eigen::Vector3d position = readVector3d(obstacle["position"]);
        Eigen::Vector3d velocity = readVector3d(obstacle["velocity"]);
        double radius = obstacle["radius"].as<double>();
        bool is_dynamic = obstacle["is_dynamic"].as<bool>();
        double angular_speed = obstacle["angular_speed"] ? obstacle["angular_speed"].as<double>() : 0.0;

        obstacles.emplace_back(name, position, velocity, radius, is_dynamic, angular_speed);
    }
    return obstacles;
}


void readAgentParameters(const YAML::Node& node, double& detect_shell_rad, double& agent_mass,
                         double& agent_radius, double& velocity_max, std::vector<double>& k_a_ee,
                         std::vector<double>& k_c_ee, std::vector<double>& k_r_ee,
                         std::vector<double>& k_r_force, std::vector<double>& k_d_ee,
                         std::vector<double>& k_manip) 
{
    detect_shell_rad = node["detect_shell_rad"].as<double>();
    agent_mass = node["agent_mass"].as<double>();
    agent_radius = node["agent_radius"].as<double>();
    velocity_max = node["velocity_max"].as<double>();
    k_a_ee = node["k_a_ee"].as<std::vector<double>>();
    k_c_ee = node["k_c_ee"].as<std::vector<double>>();
    k_r_ee = node["k_r_ee"].as<std::vector<double>>();
    k_r_force = node["k_r_force"].as<std::vector<double>>();
    k_d_ee = node["k_d_ee"].as<std::vector<double>>();
    k_manip = node["k_manip"].as<std::vector<double>>();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cf_agent_demo");
    ros::NodeHandle nh;

    ROS_INFO("CF_ANGENTS_MANAGER_DEMO");

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Point>("agent_position", 10);
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float64>("distance_to_goal", 10);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("agent_twist", 10); 
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Read from YAML files
    std::string package_path = ros::package::getPath("multi_agent_vector_fields");
    YAML::Node start_goal = YAML::LoadFile(package_path + "/config/start_goal.yaml");
    YAML::Node obstacles_yaml = YAML::LoadFile(package_path + "/config/obstacles.yaml");
    YAML::Node agent_parameters = YAML::LoadFile(package_path + "/config/agent_parameters.yaml");

    Eigen::Vector3d start_pos = readVector3d(start_goal["start_pos"]);
    Eigen::Vector3d goal_pos = readVector3d(start_goal["goal_pos"]);

    ROS_INFO("Start position: [%.2f, %.2f, %.2f]", start_pos.x(), start_pos.y(), start_pos.z());
    ROS_INFO("Goal position: [%.2f, %.2f, %.2f]", goal_pos.x(), goal_pos.y(), goal_pos.z());

    std::vector<Obstacle> obstacles = readObstacles(obstacles_yaml["obstacles"]);
    for (const auto& obs : obstacles)
    {
        ROS_INFO("Obstacle: %s, Position: [%.2f, %.2f, %.2f], Radius: %.2f", obs.getName().c_str(),
                 obs.getPosition().x(), obs.getPosition().y(), obs.getPosition().z(), obs.getRadius());
    }

    double detect_shell_rad, agent_mass, agent_radius, velocity_max;
    std::vector<double> k_a_ee, k_c_ee, k_r_ee, k_r_force, k_d_ee, k_manip;
    readAgentParameters(agent_parameters, detect_shell_rad, agent_mass, agent_radius, velocity_max,
                        k_a_ee, k_c_ee, k_r_ee, k_r_force, k_d_ee, k_manip);

    ROS_INFO("Agent Parameters:");
    ROS_INFO("  Detect shell radius: %.2f", detect_shell_rad);
    ROS_INFO("  Agent mass: %.2f", agent_mass);
    ROS_INFO("  Agent radius: %.2f", agent_radius);
    ROS_INFO("  Velocity max: %.2f", velocity_max);
    ROS_INFO("  k_a_ee: [%.2f, %.2f, %.2f, %.2f]", k_a_ee[0], k_a_ee[1], k_a_ee[2], k_a_ee[3]);
    ROS_INFO("  k_c_ee: [%.2f, %.2f, %.2f, %.2f]", k_c_ee[0], k_c_ee[1], k_c_ee[2], k_c_ee[3]);
    ROS_INFO("  k_r_ee: [%.2f, %.2f, %.2f, %.2f]", k_r_ee[0], k_r_ee[1], k_r_ee[2], k_r_ee[3]);
    ROS_INFO("  k_r_force: [%.2f, %.2f, %.2f, %.2f]", k_r_force[0], k_r_force[1], k_r_force[2], k_r_force[3]);
    ROS_INFO("  k_d_ee: [%.2f, %.2f, %.2f, %.2f]", k_d_ee[0], k_d_ee[1], k_d_ee[2], k_d_ee[3]);
    ROS_INFO("  k_manip: [%.2f, %.2f, %.2f, %.2f]", k_manip[0], k_manip[1], k_manip[2], k_manip[3]);
    ROS_INFO("----------------------------------");
    

    CfManager cf_manager(start_pos, goal_pos, 0.1, obstacles, k_a_ee, k_c_ee, k_r_ee, k_d_ee, k_r_force, k_manip);

    ros::Rate rate(10);
    bool planning_active = true;
    bool open_loop = false;

    // Visualization for Real Traj 
    visualization_msgs::Marker trajectory_marker;
    trajectory_marker.header.frame_id = "map";
    trajectory_marker.ns = "cf_agent_demo_trajectory";
    trajectory_marker.id = 101;
    trajectory_marker.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_marker.action = visualization_msgs::Marker::ADD;
    trajectory_marker.scale.x = 0.1;
    trajectory_marker.color.r = 1.0;
    trajectory_marker.color.g = 1.0;
    trajectory_marker.color.b = 0.0;
    trajectory_marker.color.a = 1.0;


    while (ros::ok()) {
        if (planning_active) {
            double start_plan_timestamp = ros::Time::now().toSec();

            // visual goal and start 
            visualizeMarker(marker_pub, start_pos, 0, "cf_agent_demo", "map", 0.5, 0.0, 1.0, 0.0, 1.0);
            visualizeMarker(marker_pub, goal_pos, 1, "cf_agent_demo", "map", 0.5, 1.0, 0.0, 0.0, 1.0);

            for (auto& obs : obstacles) 
            {
            obs.updatePosition(0.1);  // for dynamics obstacles
            // for (const auto& obs : obstacles) 
            // {
            //  ROS_INFO("Obstacle %s position: [%.2f, %.2f, %.2f]",
            //  obs.getName().c_str(),
            //  obs.getPosition().x(),
            //  obs.getPosition().y(),
            //  obs.getPosition().z());
            //  ROS_INFO("Obstacle %s dynamic: %s, angular speed: %.2f",
            //  obs.getName().c_str(),
            //  obs.isDynamic() ? "true" : "false",
            //  obs.getAngularSpeed());
            // }

            }

            // visual obstacles 
            for (size_t i = 0; i < obstacles.size(); ++i) 
            {
                visualizeMarker(marker_pub, obstacles[i].getPosition(), static_cast<int>(i + 10),
                                "cf_agent_demo_obstacles", "map", obstacles[i].getRadius() * 2.0,
                                0.0, 0.0, 1.0, 1.0);
            }

            // Set Agents first pos
            if (!open_loop) {
                cf_manager.setRealEEAgentPosition(start_pos);
                open_loop = true;
            }

            // killed
            cf_manager.stopPrediction();

            // evaluaaiton
            int best_agent_id = cf_manager.evaluateAgents(obstacles, 1.0, 1.0, 1.0, 1.0, Eigen::Matrix<double, 6, 1>::Zero());
            ROS_INFO("Best agent ID: %d", best_agent_id);

            // visual all paths
            const auto& predicted_paths = cf_manager.getPredictedPaths();
            for (size_t i = 0; i < predicted_paths.size(); ++i) 
            {
                const auto& path = predicted_paths[i];
                if (path.size() > 2) 
                {
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
                    path_marker.pose.orientation.w = 1.0;

                    for (const Eigen::Vector3d& point : path) 
                    {
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

            // visual current agent
            visualizeMarker(marker_pub, current_agent_pos, 100, "cf_agent_demo_agents", "map", 0.5, 1.0, 1.0, 0.0, 1.0);


            // upgrade real traj
            geometry_msgs::Point trajectory_point;
            trajectory_point.x = current_agent_pos.x();
            trajectory_point.y = current_agent_pos.y();
            trajectory_point.z = current_agent_pos.z();
            trajectory_marker.points.push_back(trajectory_point);
            trajectory_marker.pose.orientation.w = 1.0;
            marker_pub.publish(trajectory_marker);


            // TF Frame 
            publishAgentFrame(tf_broadcaster, current_agent_pos);

            // move Real Agent 
            cf_manager.moveRealEEAgent(obstacles, 0.1, 1, best_agent_id);

            Eigen::Vector3d updated_position = cf_manager.getNextPosition();
            //ROS_INFO("After moveRealEEAgent, position: [%.2f, %.2f, %.2f]", updated_position.x(), updated_position.y(), updated_position.z());

            double end_plan_timestamp = ros::Time::now().toSec();
            ROS_INFO("Planning time: %.3f seconds", end_plan_timestamp - start_plan_timestamp);

            // next circle 
            cf_manager.resetEEAgents(updated_position, cf_manager.getNextVelocity(), obstacles);
            cf_manager.startPrediction();

            // post agent postion 
            geometry_msgs::Point pos_msg;
            pos_msg.x = cf_manager.getNextPosition().x();
            pos_msg.y = cf_manager.getNextPosition().y();
            pos_msg.z = cf_manager.getNextPosition().z();
            pos_pub.publish(pos_msg);

            // post dist
            std_msgs::Float64 dist_msg;
            dist_msg.data = cf_manager.getDistFromGoal();
            dist_pub.publish(dist_msg);

            // post agent twists
            geometry_msgs::TwistStamped twist_msg;
            Eigen::Vector3d current_velocity = cf_manager.getNextVelocity();
            twist_msg.header.stamp = ros::Time::now();
            twist_msg.header.frame_id = "agent_frame";
            twist_msg.twist.linear.x = current_velocity.x();
            twist_msg.twist.linear.y = current_velocity.y();
            twist_msg.twist.linear.z = current_velocity.z();
            twist_msg.twist.angular.x = 0.0;
            twist_msg.twist.angular.y = 0.0;
            twist_msg.twist.angular.z = 0.0;
            twist_pub.publish(twist_msg);
        } 
        else 
        {
            ROS_INFO_STREAM("No active.");
            cf_manager.setInitialPosition(start_pos);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}