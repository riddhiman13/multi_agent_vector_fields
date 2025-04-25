#ifndef MULTI_AGENT_VECTOR_FIELDS_PARAMETER_HELPER_H
#define MULTI_AGENT_VECTOR_FIELDS_PARAMETER_HELPER_H

#include <ros/ros.h>
#include <string>
#include <vector>
#include <sstream>

// General template
template <typename T>
void readParamWithDisplay(ros::NodeHandle& node_handle, const std::string& node_name, const std::string& param_name, T& param_variable) {
    if (!node_handle.getParam(param_name, param_variable)) {
        ROS_ERROR_STREAM(node_name << ": Could not get parameter " << param_name);
    } else {
        ROS_INFO_STREAM(node_name << ": Getting parameter " << param_name << ": " << param_variable);
    }
}

// Specialization for std::vector<double>
template <>
void readParamWithDisplay<std::vector<double>>(ros::NodeHandle& node_handle, const std::string& node_name, const std::string& param_name, std::vector<double>& param_variable) {
    if (!node_handle.getParam(param_name, param_variable)) {
        ROS_ERROR_STREAM(node_name << ": Could not get parameter " << param_name);
    } else {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < param_variable.size(); ++i) {
            oss << param_variable[i];
            if (i != param_variable.size() - 1) oss << ", ";
        }
        oss << "]";
        ROS_INFO_STREAM(node_name << ": Getting parameter " << param_name << ": " << oss.str());
    }
}

#endif //MULTI_AGENT_VECTOR_FIELDS_PARAMETER_HELPER_H