#ifndef MULTI_AGENT_VECTOR_FIELDS_VISUALIZE_HELPER_H
#define MULTI_AGENT_VECTOR_FIELDS_VISUALIZE_HELPER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Geometry>

namespace multi_agent_vector_fields {

/**
 * @brief Visualizes a marker in RViz.
 * 
 * @param marker_pub The ROS publisher to publish the marker.
 * @param position The position of the marker.
 * @param orientation The orientation of the marker.
 * @param id The ID of the marker.
 * @param ns The namespace of the marker.
 * @param frame_id The frame ID of the marker.
 * @param scale The scale of the marker.
 * @param r The red color component of the marker.
 * @param g The green color component of the marker.
 * @param b The blue color component of the marker.
 * @param a The alpha (transparency) component of the marker.
 * @param type The type of the marker (default is SPHERE).
 */
void visualizeMarker(ros::Publisher& marker_pub, 
                     const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, int id, const std::string& ns, const std::string& frame_id, 
                     double scale, double r, double g, double b, double a, int type = visualization_msgs::Marker::SPHERE);

/**
 * @brief Publishes the agent's frame transform.
 * 
 * @param tf_broadcaster The TF broadcaster to publish the transform.
 * @param position The position of the agent.
 * @param orientation The orientation of the agent.
 */
void publishAgentFrame(tf2_ros::TransformBroadcaster& tf_broadcaster, 
                       const Eigen::Vector3d& position, 
                       const Eigen::Quaterniond& orientation);

/**
 * @brief Publishes a frame transform.
 * 
 * @param tf_broadcaster The TF broadcaster to publish the transform.
 * @param position The position of the frame.
 * @param orientation The orientation of the frame.
 * @param frame_id The ID of the frame.
 */
void publishFrame(tf2_ros::TransformBroadcaster& tf_broadcaster, 
                  const Eigen::Vector3d& position, 
                  const Eigen::Quaterniond& orientation, 
                  const std::string& frame_id);

void publishPathMarker(const std::vector<Eigen::Vector3d>& path, 
                    ros::Publisher& marker_pub, int id, bool is_best_agent);                   

                    
void publishPathMarkers(const std::vector<std::vector<Eigen::Vector3d>>& predicted_paths, 
                        ros::Publisher& marker_pub, 
                        int best_agent_id);

} // namespace multi_agent_vector_fields



#endif // MULTI_AGENT_VECTOR_FIELDS_VISUALIZE_HELPER_H