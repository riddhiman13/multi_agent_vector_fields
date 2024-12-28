/*
   Class for obstacle information
*/
#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include "eigen3/Eigen/Dense"

namespace ghostplanner {
namespace cfplanner {
class Obstacle {
   private:
    std::string name_;
    Eigen::Vector3d pos_;
    Eigen::Vector3d vel_;
    double rad_;
    Eigen::Vector3d center_; // For dynamic obstacles
    bool is_dynamic_; 
    double angular_speed_; // Angular speed
    int direction_; // 顺时针或逆时针

   public:
    // Constructors
    Obstacle(const std::string& name, const Eigen::Vector3d& pos,
             const Eigen::Vector3d& vel, double rad, bool is_dynamic , // = false
             double angular_speed ) // = 0
        : name_{name}, pos_{pos}, vel_{vel}, rad_{rad}, center_{pos},
          is_dynamic_{is_dynamic}, angular_speed_{angular_speed} {direction_  = (std::rand() % 2 == 0) ? -1 : 1;};

    Obstacle(const Eigen::Vector3d& pos, double rad)
        : pos_{pos}, rad_{rad}, vel_{0, 0, 0}, name_{""}, center_{pos},
          is_dynamic_{false}, angular_speed_{0.0} {};

    Obstacle(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
             double rad, bool is_dynamic = false, double angular_speed = 0.0)
        : pos_{pos}, rad_{rad}, vel_{vel}, name_{""}, center_{pos},
          is_dynamic_{is_dynamic}, angular_speed_{angular_speed} {};

    Obstacle() : pos_{0, 0, 0}, rad_{0}, vel_{0, 0, 0}, name_{""},
                 center_{0, 0, 0}, is_dynamic_{false}, angular_speed_{0.0} {};

    
    std::string getName() const { return name_; };
    Eigen::Vector3d getPosition() const { return pos_; };
    void setPosition(const Eigen::Vector3d& pos) { pos_ = pos; }
    void setVelocity(const Eigen::Vector3d& vel) { vel_ = vel; }
    Eigen::Vector3d getVelocity() const { return vel_; };
    double getRadius() const { return rad_; };
    void setDynamic(bool is_dynamic) { is_dynamic_ = is_dynamic; }
    bool isDynamic() const { return is_dynamic_; }
    void setCenter(const Eigen::Vector3d& center) { center_ = center; }
    Eigen::Vector3d getCenter() const { return center_; }
    void setAngularSpeed(double angular_speed) { angular_speed_ = angular_speed; }
    double getAngularSpeed() const { return angular_speed_; }

    // Update position for dynamic obstacles
void updatePosition(double delta_time) 
{
    if (is_dynamic_) {
        // 使用障碍物的中心位置（即 YAML 中的 position）作为圆心
        if (center_.isZero()) 
        {
            center_ = pos_; // 初始化圆心为初始位置
        }

        double angle = direction_* angular_speed_ * delta_time;
     
        Eigen::Vector3d offset = pos_ - center_;
        if (offset.norm() < 1e-6) 
        {
            offset = Eigen::Vector3d(rad_, 0, 0); 
        }
        
        Eigen::Matrix3d rotation;
        rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY());
        Eigen::Vector3d rotated_offset = rotation * offset;
        
        pos_ = center_ + rotated_offset;
    } 
    else 
    {
        // 静态障碍物只按线性速度移动
        pos_ += vel_ * delta_time;
    }
}

};
}  // namespace cfplanner
}  // namespace ghostplanner
