#ifndef HX_POS_CONTROLLER_PID_H
#define HX_POS_CONTROLLER_PID_H

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Eigen>
#include <hx_msgs/UAVState.h>

#include "math_utils.h"
#include "controller_utils.h"
#include "geometry_utils.h"
#include "printf_utils.h"

using namespace std;
 
class pos_controller_PID
{
    public:
        pos_controller_PID(){};

        void init(ros::NodeHandle& nh);

        void set_desired_state(const Desired_State& des)
        {
            desired_state = des;
        }

        void set_current_state(const hx_msgs::UAVState& state)
        {
            uav_state = state;

            for(int i=0; i<3; i++)
            {
                current_state.pos(i) = uav_state.position[i];
                current_state.vel(i) = uav_state.velocity[i];
            }

            current_state.q.w() = uav_state.attitude_q.w;
            current_state.q.x() = uav_state.attitude_q.x;
            current_state.q.y() = uav_state.attitude_q.y;
            current_state.q.z() = uav_state.attitude_q.z; 

            current_state.yaw = geometry_utils::get_yaw_from_quaternion(current_state.q);
        }

        void printf_param();
        void printf_result();
        Eigen::Vector4d update(float controller_hz);

    private:
        Ctrl_Param_PID ctrl_param;
        Desired_State desired_state;
        Current_State current_state;
        hx_msgs::UAVState uav_state; 
        Eigen::Vector3d F_des;

        Tracking_Error_Evaluation tracking_error;

        Eigen::Vector3d int_e_v;            // Velocity integral error
        Eigen::Quaterniond u_q_des;         // Desired attitude quaternion
        Eigen::Vector4d u_att;              // Desired attitude [roll, pitch, yaw, throttle]
};

void pos_controller_PID::init(ros::NodeHandle& nh)
{
    // Control parameters initialization
    ctrl_param.Kp.setZero();
    ctrl_param.Kv.setZero();
    ctrl_param.Kvi.setZero();
    ctrl_param.Ka.setZero();
    
    // UAV physical parameters
    nh.param<float>("pid_gain/quad_mass" , ctrl_param.quad_mass, 1.0f);
    nh.param<float>("pid_gain/hov_percent" , ctrl_param.hov_percent, 0.5f);
    
    // Integral limits
    nh.param<float>("pid_gain/pxy_int_max"  , ctrl_param.int_max[0], 0.5);
    nh.param<float>("pid_gain/pxy_int_max"  , ctrl_param.int_max[1], 0.5);
    nh.param<float>("pid_gain/pz_int_max"   , ctrl_param.int_max[2], 0.5);
    
    // PID gains
    nh.param<double>("pid_gain/Kp_xy", ctrl_param.Kp(0,0), 2.0f);
    nh.param<double>("pid_gain/Kp_xy", ctrl_param.Kp(1,1), 2.0f);
    nh.param<double>("pid_gain/Kp_z" , ctrl_param.Kp(2,2), 2.0f);
    nh.param<double>("pid_gain/Kv_xy", ctrl_param.Kv(0,0), 2.0f);
    nh.param<double>("pid_gain/Kv_xy", ctrl_param.Kv(1,1), 2.0f);
    nh.param<double>("pid_gain/Kv_z" , ctrl_param.Kv(2,2), 2.0f);
    nh.param<double>("pid_gain/Kvi_xy", ctrl_param.Kvi(0,0), 0.3f);
    nh.param<double>("pid_gain/Kvi_xy", ctrl_param.Kvi(1,1), 0.3f);
    nh.param<double>("pid_gain/Kvi_z" , ctrl_param.Kvi(2,2), 0.3f);
    nh.param<float>("pid_gain/tilt_angle_max" , ctrl_param.tilt_angle_max, 10.0f);
    
    ctrl_param.g << 0.0, 0.0, 9.8;

    printf_param();
}

Eigen::Vector4d pos_controller_PID::update(float controller_hz)
{
    Eigen::Vector3d e_pos, e_vel;
    
    // Position error
    e_pos = desired_state.pos - current_state.pos;
    
    // Velocity error
    e_vel = desired_state.vel - current_state.vel;
    
    // Velocity integral
    int_e_v += e_vel / controller_hz;
    
    // Anti-windup
    for(int i = 0; i < 3; i++)
    {
        int_e_v(i) = math_utils::constrain_function(int_e_v(i), -ctrl_param.int_max[i], ctrl_param.int_max[i]);
    }
    
    // Desired force calculation
    F_des = ctrl_param.Kp * e_pos + ctrl_param.Kv * e_vel + ctrl_param.Kvi * int_e_v + 
            ctrl_param.quad_mass * desired_state.acc + ctrl_param.quad_mass * ctrl_param.g;
    
    // Get desired attitude from force
    geometry_utils::get_desired_attitude_from_desired_force(F_des, desired_state.yaw, u_q_des);
    
    // Convert to euler angles
    Eigen::Vector3d desired_att = geometry_utils::quaternion_to_euler(u_q_des);
    
    // Apply angle limits
    desired_att(0) = math_utils::constrain_function(desired_att(0), -ctrl_param.tilt_angle_max*M_PI/180.0, ctrl_param.tilt_angle_max*M_PI/180.0);
    desired_att(1) = math_utils::constrain_function(desired_att(1), -ctrl_param.tilt_angle_max*M_PI/180.0, ctrl_param.tilt_angle_max*M_PI/180.0);
    
    // Calculate throttle
    double throttle = F_des.norm() / (ctrl_param.quad_mass * 9.8);
    throttle = math_utils::constrain_function(throttle, 0.1, 0.95);
    
    // Pack output
    u_att(0) = desired_att(0);  // roll
    u_att(1) = desired_att(1);  // pitch
    u_att(2) = desired_att(2);  // yaw
    u_att(3) = throttle;        // throttle
    
    // Update tracking error evaluation
    tracking_error.pos_error = e_pos;
    tracking_error.vel_error = e_vel;
    tracking_error.pos_error_norm = e_pos.norm();
    tracking_error.vel_error_norm = e_vel.norm();
    
    return u_att;
}

void pos_controller_PID::printf_param()
{
    cout << "PID Controller Parameters:" << endl;
    cout << "quad_mass: " << ctrl_param.quad_mass << endl;
    cout << "hov_percent: " << ctrl_param.hov_percent << endl;
    cout << "Kp: " << endl << ctrl_param.Kp << endl;
    cout << "Kv: " << endl << ctrl_param.Kv << endl;
    cout << "Kvi: " << endl << ctrl_param.Kvi << endl;
    cout << "tilt_angle_max: " << ctrl_param.tilt_angle_max << endl;
}

void pos_controller_PID::printf_result()
{
    cout << "PID Controller Output:" << endl;
    cout << "Desired Force: [" << F_des(0) << ", " << F_des(1) << ", " << F_des(2) << "]" << endl;
    cout << "Attitude Command: [" << u_att(0)*180/M_PI << ", " << u_att(1)*180/M_PI << ", " << u_att(2)*180/M_PI << ", " << u_att(3) << "]" << endl;
    cout << "Position Error: " << tracking_error.pos_error_norm << endl;
    cout << "Velocity Error: " << tracking_error.vel_error_norm << endl;
}

#endif