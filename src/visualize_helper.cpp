#include "multi_agent_vector_fields/visualize_helper.h"


void multi_agent_vector_fields::visualizeMarker(ros::Publisher &marker_pub, 
                     const Eigen::Vector3d &position, const Eigen::Quaterniond &orientation, int id, const std::string &ns, const std::string &frame_id, 
                     double scale, double r, double g, double b, double a, int type)
{
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
    marker.pose.orientation.x = orientation.x();
    marker.pose.orientation.y = orientation.y();
    marker.pose.orientation.z = orientation.z();
    marker.pose.orientation.w = orientation.w();
    marker_pub.publish(marker);
}

void multi_agent_vector_fields::publishAgentFrame(tf2_ros::TransformBroadcaster &tf_broadcaster, 
                                                const Eigen::Vector3d &position, 
                                                const Eigen::Quaterniond &orientation)
{
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "map";
    transform.child_frame_id = "agent_frame";

    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();
    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();

    tf_broadcaster.sendTransform(transform);
}

void multi_agent_vector_fields::publishFrame(tf2_ros::TransformBroadcaster &tf_broadcaster, 
                                            const Eigen::Vector3d &position, 
                                            const Eigen::Quaterniond &orientation, 
                                            const std::string &frame_id)
{
    geometry_msgs::TransformStamped transform;
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "map";
    transform.child_frame_id = frame_id;

    transform.transform.translation.x = position.x();
    transform.transform.translation.y = position.y();
    transform.transform.translation.z = position.z();

    transform.transform.rotation.x = orientation.x();
    transform.transform.rotation.y = orientation.y();
    transform.transform.rotation.z = orientation.z();
    transform.transform.rotation.w = orientation.w();

    tf_broadcaster.sendTransform(transform);
}