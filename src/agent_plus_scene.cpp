#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/Float64.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>

#include "multi_agent_vector_fields/cf_manager.h"
#include "multi_agent_vector_fields/obstacle.h"
#include "multi_agent_vector_fields/visualize_helper.h"

#include <yaml-cpp/yaml.h>

using namespace ghostplanner::cfplanner;

std::vector<Obstacle> obstacles;

Eigen::Vector3d readVector3d(const YAML::Node& node) 
{
    auto vec = node.as<std::vector<double>>();
    return Eigen::Vector3d(vec[0], vec[1], vec[2]);
}

Eigen::Quaterniond readQuaternion(const YAML::Node& node) {
    auto quat = node.as<std::vector<double>>();
    return Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]);
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
                         double& agent_radius, double& velocity_max, double& approach_dist,
                         std::vector<double>& k_a_ee, std::vector<double>& k_c_ee, 
                         std::vector<double>& k_r_ee, std::vector<double>& k_r_force, 
                         std::vector<double>& k_d_ee, std::vector<double>& k_manip) 
{
    detect_shell_rad = node["detect_shell_rad"].as<double>();
    agent_mass = node["agent_mass"].as<double>();
    agent_radius = node["agent_radius"].as<double>();
    velocity_max = node["velocity_max"].as<double>();
    approach_dist = node["approach_dist"].as<double>();
    k_a_ee = node["k_a_ee"].as<std::vector<double>>();
    k_c_ee = node["k_c_ee"].as<std::vector<double>>();
    k_r_ee = node["k_r_ee"].as<std::vector<double>>();
    k_r_force = node["k_r_force"].as<std::vector<double>>();
    k_d_ee = node["k_d_ee"].as<std::vector<double>>();
    k_manip = node["k_manip"].as<std::vector<double>>();
}

void planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr& msg) {
    obstacles.clear();
    for (const auto& obj : msg->world.collision_objects) {
        if (obj.primitives.empty()) continue;

        const auto& pose = obj.primitive_poses[0];
        Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
        double radius = (obj.primitives[0].type == shape_msgs::SolidPrimitive::SPHERE) ? 
                         obj.primitives[0].dimensions[0] : 0.5;

        obstacles.emplace_back(obj.id, position, Eigen::Vector3d(0, 0, 0), radius, false, 0.0);
    }

    ROS_INFO("Load %zu obstacles from topic", obstacles.size());
}

void waitForFirstPlanningScene() {
    ros::Rate rate(10);
    while (ros::ok() && obstacles.empty()) {
        ros::spinOnce();
        ROS_INFO_THROTTLE(1, "Waiting for first planning_scene message...");
        rate.sleep();
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "cf_agent_demo");
    ros::NodeHandle nh;

    ROS_INFO("CF_ANGENTS_MANAGER_DEMO");

    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Point>("agent_position", 10);
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float64>("distance_to_goal", 10);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("agent_twist_local", 10);
    ros::Publisher twist_pub_2 = nh.advertise<geometry_msgs::TwistStamped>("agent_twist_global", 10); 
    tf2_ros::TransformBroadcaster tf_broadcaster;

    // Read from YAML files
    std::string package_path = ros::package::getPath("multi_agent_vector_fields");
    YAML::Node start_goal = YAML::LoadFile(package_path + "/config/start_goal.yaml");
    //YAML::Node obstacles_yaml = YAML::LoadFile(package_path + "/config/obstacles_1.yaml");
    YAML::Node agent_parameters = YAML::LoadFile(package_path + "/config/agent_parameters.yaml");

    Eigen::Vector3d start_pos = readVector3d(start_goal["start_pos"]);
    Eigen::Vector3d goal_pos = readVector3d(start_goal["goal_pos"]);
    Eigen::Quaterniond start_orientation = readQuaternion(start_goal["start_orientation"]);
    Eigen::Quaterniond goal_orientation = readQuaternion(start_goal["goal_orientation"]);

    ROS_INFO("Start position: [%.2f, %.2f, %.2f]", start_pos.x(), start_pos.y(), start_pos.z());
    ROS_INFO("Goal position: [%.2f, %.2f, %.2f]", goal_pos.x(), goal_pos.y(), goal_pos.z());
    ROS_INFO("Start orientation: [%.2f, %.2f, %.2f, %.2f]", start_orientation.w(), start_orientation.x(), start_orientation.y(), start_orientation.z());
    ROS_INFO("Goal orientation: [%.2f, %.2f, %.2f, %.2f]", goal_orientation.w(), goal_orientation.x(), goal_orientation.y(), goal_orientation.z());


    // std::vector<Obstacle> obstacles = readObstacles(obstacles_yaml["obstacles"]);
    //std::vector<Obstacle> obstacles;

    //TODO NEED subscirbe the topic Planning Scene and conver
    ros::Subscriber planning_scene_sub = nh.subscribe("/planning_scene", 1, planningSceneCallback);
    waitForFirstPlanningScene(); 
    /*for (const auto& obs : obstacles)
    {
        ROS_INFO("Obstacle: %s, Position: [%.2f, %.2f, %.2f], Radius: %.2f", obs.getName().c_str(),
                 obs.getPosition().x(), obs.getPosition().y(), obs.getPosition().z(), obs.getRadius());
    }
    */

    double detect_shell_rad, agent_mass, agent_radius, velocity_max, approach_dist;
    std::vector<double> k_a_ee, k_c_ee, k_r_ee, k_r_force, k_d_ee, k_manip;
    readAgentParameters(agent_parameters, detect_shell_rad, agent_mass, agent_radius, velocity_max, approach_dist,
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
    

    CfManager cf_manager(start_pos, goal_pos, 
                        0.1, obstacles, 
                        k_a_ee, k_c_ee, 
                        k_r_ee, k_d_ee, 
                        k_manip, k_r_force, 
                        start_orientation, 
                        goal_orientation,
                        velocity_max, detect_shell_rad,
                        agent_mass, agent_radius);

    ros::Rate rate(10);
    bool planning_active = true;
    bool open_loop = false;

    // Visualization for Real Traj 
    std::vector<Eigen::Vector3d> trajectory_history;
    
    while (ros::ok()) 
    {
        ros::spinOnce();
        if (planning_active) {
            // Set Agents first pos
            if (!open_loop) 
            {
                cf_manager.setRealEEAgentPosition(start_pos);
                open_loop = true;
            }

            // killed
            cf_manager.stopPrediction();

            // evaluaaiton
            int best_agent_id = cf_manager.evaluateAgents(obstacles, 1.0, 1.0, 1.0, 1.0, Eigen::Matrix<double, 6, 1>::Zero());
            ROS_INFO("Best agent ID: %d", best_agent_id);

            // visualize predicted paths
            const auto& predicted_paths = cf_manager.getPredictedPaths();

            Eigen::Vector3d current_agent_pos = cf_manager.getNextPosition();
            ROS_INFO("Current position: [%.2f, %.2f, %.2f]", current_agent_pos.x(), current_agent_pos.y(), current_agent_pos.z());
            
            Eigen::Quaterniond current_agent_orientation = cf_manager.getNextOrientation();
            ROS_INFO("Current orientation: [w=%.2f, x=%.2f, y=%.2f, z=%.2f]",current_agent_orientation.w(), current_agent_orientation.x(),
                                                                             current_agent_orientation.y(), current_agent_orientation.z());
            
            // update real traj
            trajectory_history.push_back(current_agent_pos);

            // move Real Agent 
            cf_manager.moveRealEEAgent(obstacles, 0.1, 1, best_agent_id);

            Eigen::Vector3d updated_position = cf_manager.getNextPosition();
            //ROS_INFO("After moveRealEEAgent, position: [%.2f, %.2f, %.2f]", updated_position.x(), updated_position.y(), updated_position.z());


            // next iteration 
            cf_manager.resetEEAgents(updated_position, cf_manager.getNextVelocity(), obstacles);
            cf_manager.startPrediction();

            /** visualize the data in RViz */
            // visualize goal and start as sphere
            //multi_agent_vector_fields::visualizeMarker(marker_pub, start_pos, start_orientation, 0, "multi_agent_condition", "world", 0.05, 0.0, 1.0, 0.0, 0.5);
            //multi_agent_vector_fields::visualizeMarker(marker_pub, goal_pos , goal_orientation, 1, "multi_agent_condition", "world", 0.05, 1.0, 0.0, 0.0, 0.5);
            multi_agent_vector_fields::visualizeMarker(marker_pub, current_agent_pos, Eigen::Quaterniond::Identity() , 100, 
                                                        "multi_agent_agent", "world", agent_radius*2, 1.0, 1.0, 0.0, 0.5);

            // Goal and Start Frame 
            multi_agent_vector_fields::publishFrame(tf_broadcaster, start_pos, start_orientation, "start_frame");
            multi_agent_vector_fields::publishFrame(tf_broadcaster, goal_pos, goal_orientation, "goal_frame");
            multi_agent_vector_fields::publishFrame(tf_broadcaster, current_agent_pos, current_agent_orientation, "agent_frame");

            // visualize predicted and history path
            multi_agent_vector_fields::publishMultiAgentPlannedPaths(marker_pub, predicted_paths, best_agent_id);
            multi_agent_vector_fields::publishPathMarker(marker_pub, trajectory_history, 201, "world", "multi_agent_trajectory", 
                                                        0.01, 1.0, 1.0, 0.0, 0.8);

            // visual obstacles 
            for (size_t i = 0; i < obstacles.size(); ++i) {
                multi_agent_vector_fields::visualizeMarker(marker_pub, obstacles[i].getPosition(), Eigen::Quaterniond::Identity(),
                                                            static_cast<int>(i + 10), "multi_agent_obstacles", "world", 
                                                            obstacles[i].getRadius() * 2.0, 0.6, 0.2, 0.1, 0.9);
            }

            // post agent postion 
            geometry_msgs::Point pos_msg;
            pos_msg.x = cf_manager.getNextPosition().x();
            pos_msg.y = cf_manager.getNextPosition().y();
            pos_msg.z = cf_manager.getNextPosition().z();
            pos_pub.publish(pos_msg);

            // post distance
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

            Eigen::Vector3d current_angular_velocity = cf_manager.getNextAngularVelocity();
            twist_msg.twist.angular.x = current_angular_velocity.x();
            twist_msg.twist.angular.y = current_angular_velocity.y();
            twist_msg.twist.angular.z = current_angular_velocity.z();
            twist_pub.publish(twist_msg);

            // post agent twists in global frame
            // this is actually not really a twist "transformation". We only covert the twist for its orientation.
            geometry_msgs::TwistStamped twist_msg_global;
            Eigen::Vector3d current_velocity_global = current_agent_orientation.toRotationMatrix() * current_velocity;
            twist_msg_global.header.stamp = ros::Time::now();
            twist_msg_global.header.frame_id = "world";
            twist_msg_global.twist.linear.x = current_velocity_global.x();
            twist_msg_global.twist.linear.y = current_velocity_global.y();
            twist_msg_global.twist.linear.z = current_velocity_global.z();

            Eigen::Vector3d current_angular_velocity_global = current_agent_orientation.toRotationMatrix() * current_angular_velocity;
            twist_msg_global.twist.angular.x = current_angular_velocity_global.x();
            twist_msg_global.twist.angular.y = current_angular_velocity_global.y();
            twist_msg_global.twist.angular.z = current_angular_velocity_global.z();
            twist_pub_2.publish(twist_msg_global);
        } 
        else 
        {
            ROS_INFO_STREAM("No active.");
            cf_manager.setInitialPosition(start_pos);
        }
        rate.sleep();
    }

    return 0;
}