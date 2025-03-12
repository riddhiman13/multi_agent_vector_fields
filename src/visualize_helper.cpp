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
    transform.header.frame_id = "world";
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
    transform.header.frame_id = "world";
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


void multi_agent_vector_fields::publishPathMarker(const std::vector<Eigen::Vector3d>& path, 
                                                   ros::Publisher& marker_pub, int id, bool is_best_agent) 
{
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.ns = "cf_agent_demo_paths";
    path_marker.id = id;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.04;
    path_marker.color.r = is_best_agent ? 1.0 : 0.0;
    path_marker.color.g = 0.2;
    path_marker.color.b = is_best_agent ? 0.0 : 1.0;
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

void multi_agent_vector_fields::publishPathMarkers(const std::vector<std::vector<Eigen::Vector3d>>& predicted_paths, 
                                                   ros::Publisher& marker_pub, int best_agent_id) 
{
    for (size_t i = 0; i < predicted_paths.size(); ++i) 
    {
        const auto& path = predicted_paths[i];
        if (path.size() > 2) 
        {
            publishPathMarker(path, marker_pub, static_cast<int>(i + 20), i == best_agent_id);
        }
    }
}
