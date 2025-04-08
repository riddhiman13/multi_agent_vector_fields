#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/Float64.h>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>

#include "multi_agent_vector_fields/cf_manager.h"
#include "multi_agent_vector_fields/visualize_helper.h"
#include "multi_agent_vector_fields/parameter_helper.h"

using namespace ghostplanner::cfplanner;

// Subsriber: goal
Eigen::Vector3d goal_pos_;
Eigen::Quaterniond goal_orientation_;
bool first_receive_goal_ = false;
void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    if (first_receive_goal_ == false) first_receive_goal_ = true;
    goal_pos_ = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    goal_orientation_ = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, 
                                           msg->pose.orientation.y, msg->pose.orientation.z);

    ROS_INFO("Received goal position: [%.2f, %.2f, %.2f]", goal_pos_.x(), goal_pos_.y(), goal_pos_.z());
    ROS_INFO("Received goal orientation: [%.2f, %.2f, %.2f, %.2f]", 
             goal_orientation_.w(), goal_orientation_.x(), goal_orientation_.y(), goal_orientation_.z());
}

// Subscriber: planning scene
std::vector<Obstacle> obstacles_;
bool first_receive_obstacles_ = false;
void planningSceneCallback(const moveit_msgs::PlanningScene::ConstPtr& msg) {
    if (first_receive_obstacles_ == false) first_receive_obstacles_ = true;
    obstacles_.clear();
    for (const auto& obj : msg->world.collision_objects) {
        if (obj.primitives.empty()) continue;

        const auto& pose = obj.primitive_poses[0];
        Eigen::Vector3d position(pose.position.x, pose.position.y, pose.position.z);
        double radius = (obj.primitives[0].type == shape_msgs::SolidPrimitive::SPHERE) ? 
                         obj.primitives[0].dimensions[0] : 0.5;

        obstacles_.emplace_back(obj.id, position, Eigen::Vector3d(0, 0, 0), radius, false, 0.0);
    }
}

// Subscriber: franka state
Eigen::Vector3d TCP_pos_;
Eigen::Quaterniond TCP_orientation_;
Eigen::Vector3d start_pos_(0.0, 0.0, 0.0);
Eigen::Quaterniond start_orientation_(1.0, 0.0, 0.0, 0.0);
bool first_receive_TCP_ = false;
void frankaStateCallback(const franka_msgs::FrankaState::ConstPtr& msg) 
{
    Eigen::Matrix4d TCP_transform = Eigen::Map<const Eigen::Matrix4d>(msg->O_T_EE.data());

    TCP_pos_ = TCP_transform.block<3, 1>(0, 3);
    TCP_orientation_ = Eigen::Quaterniond(Eigen::Matrix3d(TCP_transform.block<3, 3>(0, 0)));

    if (first_receive_TCP_ == false){
        first_receive_TCP_ = true;
        start_pos_ = TCP_pos_;
        start_orientation_ = TCP_orientation_;
    }
}

int main(int argc, char** argv) {
    std::string node_name = "multi_agent_scene_status";
    ros::init(argc, argv, node_name);
    ros::NodeHandle nh;

    // Load ROS parameters
    int frequency;
    double detect_shell_rad, agent_mass, agent_radius, velocity_max, approach_dist;
    std::vector<double> k_a_ee, k_c_ee, k_r_ee, k_r_force, k_d_ee, k_manip;

    readParamWithDisplay(nh, node_name, "/multi_agent_vector_fields/frequency",         frequency);
    readParamWithDisplay(nh, node_name, "/multi_agent_vector_fields/detect_shell_rad",  detect_shell_rad);
    readParamWithDisplay(nh, node_name, "/multi_agent_vector_fields/agent_mass",        agent_mass);
    readParamWithDisplay(nh, node_name, "/multi_agent_vector_fields/agent_radius",      agent_radius);
    readParamWithDisplay(nh, node_name, "/multi_agent_vector_fields/velocity_max",      velocity_max);
    readParamWithDisplay(nh, node_name, "/multi_agent_vector_fields/approach_dist",     approach_dist);

    readParamWithDisplay(nh, node_name, "/multi_agent_vector_fields/k_a_ee",    k_a_ee);
    readParamWithDisplay(nh, node_name, "/multi_agent_vector_fields/k_c_ee",    k_c_ee);
    readParamWithDisplay(nh, node_name, "/multi_agent_vector_fields/k_r_ee",    k_r_ee);
    readParamWithDisplay(nh, node_name, "/multi_agent_vector_fields/k_r_force", k_r_force);
    readParamWithDisplay(nh, node_name, "/multi_agent_vector_fields/k_d_ee",    k_d_ee);
    readParamWithDisplay(nh, node_name, "/multi_agent_vector_fields/k_manip",   k_manip);

    // Set up publishers and subscribers
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::Point>("agent_position", 10);
    ros::Publisher dist_pub = nh.advertise<std_msgs::Float64>("distance_to_goal", 10);
    ros::Publisher twist_pub = nh.advertise<geometry_msgs::TwistStamped>("agent_twist_local", 10);
    ros::Publisher twist_pub_2 = nh.advertise<geometry_msgs::TwistStamped>("agent_twist_global", 10);
    tf2_ros::TransformBroadcaster tf_broadcaster;

    ros::Subscriber goal_sub = nh.subscribe("/goal", 1, goalCallback);
    ros::Subscriber franka_state_sub = nh.subscribe("/franka_state_controller/franka_states", 1, frankaStateCallback);
    ros::Subscriber planning_scene_sub = nh.subscribe("/planning_scene", 1, planningSceneCallback);

    // Wait for the first receive of the message
    ros::Rate rate(frequency);
    while (ros::ok() && (!first_receive_goal_ || !first_receive_obstacles_ || !first_receive_TCP_)) {
        ros::spinOnce();
        ROS_INFO_STREAM_THROTTLE(1, node_name << ": Waiting for first messages:"
                                 << " goal: [" << first_receive_goal_ << "],"
                                 << " obstacles: [" << first_receive_obstacles_ << "],"
                                 << " TCP: [" << first_receive_TCP_ << "]");
        rate.sleep();
    }

    ROS_INFO_STREAM(node_name << ": start position (at first moment): " << start_pos_.transpose());
    ROS_INFO_STREAM(node_name << ": start orientation (at first moment): " << start_orientation_.coeffs().transpose());
    ROS_INFO_STREAM(node_name << ": goal position (at first moment): " << goal_pos_.transpose());
    ROS_INFO_STREAM(node_name << ": goal orientation (at first moment): " << goal_orientation_.coeffs().transpose());
        
    CfManager cf_manager(start_pos_, goal_pos_, 
                        1.0/ static_cast<double>(frequency), 
                        obstacles_, 
                        k_a_ee, k_c_ee, k_r_ee, k_d_ee, k_manip, k_r_force, 
                        start_orientation_, goal_orientation_,
                        velocity_max, detect_shell_rad, agent_mass, agent_radius);

    // Visualization for Real Traj 
    std::vector<Eigen::Vector3d> trajectory_history;

    bool planning_active = true;
    cf_manager.setRealEEAgentPosition(start_pos_);
    while (ros::ok()) 
    {
        if (planning_active) {
            // killed
            cf_manager.stopPrediction();

            // evaluaiton on best agent
            int best_agent_id = cf_manager.evaluateAgents(obstacles_, 1.0, 1.0, 1.0, 1.0, Eigen::Matrix<double, 6, 1>::Zero());

            // retrieve the data
            const auto& predicted_paths = cf_manager.getPredictedPaths();
            Eigen::Vector3d current_agent_pos = cf_manager.getNextPosition();
            Eigen::Quaterniond current_agent_orientation = cf_manager.getNextOrientation();
            trajectory_history.push_back(current_agent_pos);

            // move Real Agent 
            cf_manager.moveRealEEAgent(obstacles_, 0.1, 1, best_agent_id);
            Eigen::Vector3d updated_position = cf_manager.getNextPosition();

            // next iteration 
            cf_manager.setRealEEAgentPosition(TCP_pos_);
            cf_manager.resetEEAgents(updated_position, cf_manager.getNextVelocity(), obstacles_);
            cf_manager.startPrediction();

            /** visualize the data in RViz */
            // visualize goal and start as sphere
            //multi_agent_vector_fields::visualizeMarker(marker_pub, start_pos, start_orientation, 0, "multi_agent_condition", "world", 0.05, 0.0, 1.0, 0.0, 0.5);
            //multi_agent_vector_fields::visualizeMarker(marker_pub, goal_pos , goal_orientation, 1, "multi_agent_condition", "world", 0.05, 1.0, 0.0, 0.0, 0.5);
            multi_agent_vector_fields::visualizeMarker(marker_pub, current_agent_pos, Eigen::Quaterniond::Identity(), 100, "multi_agent_agent", 
                                                                                                            "world", agent_radius*2, 1.0, 1.0, 0.0, 0.5);

            // visualize goal, start and agent frame in tf 
            multi_agent_vector_fields::publishFrame(tf_broadcaster, start_pos_, start_orientation_, "start_frame");
            multi_agent_vector_fields::publishFrame(tf_broadcaster, goal_pos_, goal_orientation_, "goal_frame");
            multi_agent_vector_fields::publishFrame(tf_broadcaster, current_agent_pos, current_agent_orientation, "agent_frame");

            // visualize predicted and history path
            multi_agent_vector_fields::publishMultiAgentPlannedPaths(marker_pub, predicted_paths, best_agent_id);
            multi_agent_vector_fields::publishPathMarker(marker_pub, trajectory_history, 201, "world", "multi_agent_trajectory", 
                                                        0.01, 1.0, 1.0, 0.0, 0.8);

            /** publish the data as message */
            // post agent postion 
            geometry_msgs::Point pos_msg;
            pos_msg.x = current_agent_pos.x();
            pos_msg.y = current_agent_pos.y();
            pos_msg.z = current_agent_pos.z();
            pos_pub.publish(pos_msg);

            // post distance
            std_msgs::Float64 dist_msg;
            dist_msg.data = cf_manager.getDistFromGoal();
            dist_pub.publish(dist_msg);

            // post agent twists
            // this is actually not a "twist", its linear and angular velocity is not relevant.
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
            cf_manager.setInitialPosition(TCP_pos_);
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}