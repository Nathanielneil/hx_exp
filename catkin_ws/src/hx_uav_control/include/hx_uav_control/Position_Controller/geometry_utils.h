#ifndef __HX_GEOMETRY_UTILS_H
#define __HX_GEOMETRY_UTILS_H

#include <Eigen/Dense>
#include <cmath>

namespace geometry_utils {

// Convert degrees to radians
template <typename Scalar_t>
Scalar_t toRad(const Scalar_t& x) {
    return x / 180.0 * M_PI;
}

// Convert radians to degrees
template <typename Scalar_t>
Scalar_t toDeg(const Scalar_t& x) {
    return x * 180.0 / M_PI;
}

// Rotation matrix around X axis
template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> rotx(Scalar_t t) {
    Eigen::Matrix<Scalar_t, 3, 3> R;
    R(0, 0) = 1.0;
    R(0, 1) = 0.0;
    R(0, 2) = 0.0;
    R(1, 0) = 0.0;
    R(1, 1) = std::cos(t);
    R(1, 2) = -std::sin(t);
    R(2, 0) = 0.0;
    R(2, 1) = std::sin(t);
    R(2, 2) = std::cos(t);
    return R;
}

// Rotation matrix around Y axis
template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> roty(Scalar_t t) {
    Eigen::Matrix<Scalar_t, 3, 3> R;
    R(0, 0) = std::cos(t);
    R(0, 1) = 0.0;
    R(0, 2) = std::sin(t);
    R(1, 0) = 0.0;
    R(1, 1) = 1.0;
    R(1, 2) = 0;
    R(2, 0) = -std::sin(t);
    R(2, 1) = 0.0;
    R(2, 2) = std::cos(t);
    return R;
}

// Rotation matrix around Z axis
template <typename Scalar_t>
Eigen::Matrix<Scalar_t, 3, 3> rotz(Scalar_t t) {
    Eigen::Matrix<Scalar_t, 3, 3> R;
    R(0, 0) = std::cos(t);
    R(0, 1) = -std::sin(t);
    R(0, 2) = 0.0;
    R(1, 0) = std::sin(t);
    R(1, 1) = std::cos(t);
    R(1, 2) = 0.0;
    R(2, 0) = 0.0;
    R(2, 1) = 0.0;
    R(2, 2) = 1.0;
    return R;
}

// Extract yaw angle from quaternion
double get_yaw_from_quaternion(const Eigen::Quaterniond& q)
{
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    return atan2(siny_cosp, cosy_cosp);
}

// Convert quaternion to euler angles (roll, pitch, yaw)
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q)
{
    Eigen::Vector3d euler;
    
    // Roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
    double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
    euler(0) = atan2(sinr_cosp, cosr_cosp);
    
    // Pitch (y-axis rotation)
    double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
    if (abs(sinp) >= 1)
        euler(1) = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        euler(1) = asin(sinp);
    
    // Yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
    double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
    euler(2) = atan2(siny_cosp, cosy_cosp);
    
    return euler;
}

// Convert euler angles to quaternion
Eigen::Quaterniond euler_to_quaternion(const Eigen::Vector3d& euler)
{
    double roll = euler(0);
    double pitch = euler(1);
    double yaw = euler(2);
    
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    
    Eigen::Quaterniond q;
    q.w() = cr * cp * cy + sr * sp * sy;
    q.x() = sr * cp * cy - cr * sp * sy;
    q.y() = cr * sp * cy + sr * cp * sy;
    q.z() = cr * cp * sy - sr * sp * cy;
    
    return q;
}

// Get desired attitude from desired force vector
void get_desired_attitude_from_desired_force(const Eigen::Vector3d& desired_force, 
                                           double desired_yaw,
                                           Eigen::Quaterniond& desired_quaternion)
{
    // Calculate desired thrust direction (normalized)
    Eigen::Vector3d thrust_direction = desired_force.normalized();
    
    // Calculate desired roll and pitch from thrust direction
    double desired_pitch = asin(-thrust_direction(0));  // Negative because of ENU convention
    double desired_roll = atan2(thrust_direction(1), thrust_direction(2));
    
    // Create desired attitude
    Eigen::Vector3d desired_euler(desired_roll, desired_pitch, desired_yaw);
    desired_quaternion = euler_to_quaternion(desired_euler);
}

// Convert YPR (Yaw-Pitch-Roll) to rotation matrix
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr_to_R(const Eigen::DenseBase<Derived>& ypr) {
    typename Derived::Scalar y = ypr(0);  // yaw
    typename Derived::Scalar p = ypr(1);  // pitch  
    typename Derived::Scalar r = ypr(2);  // roll
    
    return rotz(y) * roty(p) * rotx(r);
}

// Convert rotation matrix to YPR (Yaw-Pitch-Roll)
template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, 3, 1> R_to_ypr(const Eigen::DenseBase<Derived>& R) {
    Eigen::Matrix<typename Derived::Scalar, 3, 1> ypr;
    
    // Extract yaw, pitch, roll from rotation matrix
    ypr(0) = atan2(R(1,0), R(0,0));  // yaw
    ypr(1) = atan2(-R(2,0), sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2)));  // pitch
    ypr(2) = atan2(R(2,1), R(2,2));  // roll
    
    return ypr;
}

} // namespace geometry_utils

#endif