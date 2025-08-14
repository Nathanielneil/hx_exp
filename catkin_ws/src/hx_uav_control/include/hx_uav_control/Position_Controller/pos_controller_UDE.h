#ifndef HX_POS_CONTROLLER_UDE_H
#define HX_POS_CONTROLLER_UDE_H

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Eigen>
#include <bitset>
#include <hx_msgs/UAVState.h>

#include "geometry_utils.h"
#include "controller_utils.h"
#include "printf_utils.h"

using namespace std;

class pos_controller_UDE
{
    public:
        pos_controller_UDE(){};

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
        Ctrl_Param_UDE ctrl_param;
        Desired_State desired_state;
        Current_State current_state;
        hx_msgs::UAVState uav_state;

        Tracking_Error_Evaluation tracking_error;

        // u_l for nominal control (PD), u_d for UDE control (disturbance estimator)
        Eigen::Vector3d u_l, u_d;
        Eigen::Vector3d integral;
        Eigen::Vector3d F_des;
        Eigen::Vector4d u_att;              // Desired attitude [roll, pitch, yaw, throttle]
        
        // UDE specific variables
        Eigen::Vector3d disturbance_estimation;
        Eigen::Vector3d previous_disturbance;
        Eigen::Vector3d filter_state;
};

void pos_controller_UDE::init(ros::NodeHandle& nh)
{
    ctrl_param.Kp.setZero();
    ctrl_param.Kd.setZero();
    
    // Physical parameters
    nh.param<float>("ude_gain/quad_mass"   , ctrl_param.quad_mass, 1.0f);
    nh.param<float>("ude_gain/hov_percent" , ctrl_param.hov_percent, 0.5f);
    nh.param<float>("ude_gain/pxy_int_max" , ctrl_param.int_max[0], 1.0);
    nh.param<float>("ude_gain/pxy_int_max" , ctrl_param.int_max[1], 1.0);
    nh.param<float>("ude_gain/pz_int_max"  , ctrl_param.int_max[2], 1.0);
    
    ctrl_param.g << 0.0, 0.0, 9.8;

    // Control gains
    nh.param<double>("ude_gain/Kp_xy", ctrl_param.Kp(0,0), 0.5f);
    nh.param<double>("ude_gain/Kp_xy", ctrl_param.Kp(1,1), 0.5f);
    nh.param<double>("ude_gain/Kp_z" , ctrl_param.Kp(2,2), 0.5f);
    nh.param<double>("ude_gain/Kd_xy", ctrl_param.Kd(0,0), 0.3f);
    nh.param<double>("ude_gain/Kd_xy", ctrl_param.Kd(1,1), 0.3f);
    nh.param<double>("ude_gain/Kd_z" , ctrl_param.Kd(2,2), 0.3f);
    
    // UDE filter parameter
    nh.param<double>("ude_gain/T_ude", ctrl_param.T_ude, 0.1);
    nh.param<float>("ude_gain/tilt_angle_max" , ctrl_param.tilt_angle_max, 10.0f);

    // Initialize UDE variables
    disturbance_estimation.setZero();
    previous_disturbance.setZero();
    filter_state.setZero();

    printf_param();
}

Eigen::Vector4d pos_controller_UDE::update(float controller_hz)
{
    Eigen::Vector3d e_pos, e_vel;
    float dt = 1.0 / controller_hz;
    
    // Position and velocity errors
    e_pos = desired_state.pos - current_state.pos;
    e_vel = desired_state.vel - current_state.vel;
    
    // Baseline controller (PD)
    u_l = ctrl_param.Kp * e_pos + ctrl_param.Kd * e_vel + 
          ctrl_param.quad_mass * desired_state.acc + ctrl_param.quad_mass * ctrl_param.g;
    
    // Disturbance estimation using UDE filter
    Eigen::Vector3d estimated_acceleration = current_state.vel; // This should be measured acceleration
    Eigen::Vector3d nominal_acceleration = u_l / ctrl_param.quad_mass - ctrl_param.g;
    Eigen::Vector3d disturbance_raw = estimated_acceleration - nominal_acceleration;
    
    // Low-pass filter for disturbance estimation
    float alpha = dt / (ctrl_param.T_ude + dt);
    disturbance_estimation = (1 - alpha) * previous_disturbance + alpha * disturbance_raw;
    previous_disturbance = disturbance_estimation;
    
    // UDE compensation
    u_d = ctrl_param.quad_mass * disturbance_estimation;
    
    // Total desired force
    F_des = u_l + u_d;
    
    // Get desired attitude from force
    Eigen::Quaterniond u_q_des;
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
    
    // Update tracking error
    tracking_error.input_error(e_pos, e_vel);
    
    return u_att;
}

void pos_controller_UDE::printf_param()
{
    cout << "UDE Controller Parameters:" << endl;
    cout << "quad_mass: " << ctrl_param.quad_mass << endl;
    cout << "hov_percent: " << ctrl_param.hov_percent << endl;
    cout << "T_ude: " << ctrl_param.T_ude << endl;
    cout << "Kp: " << endl << ctrl_param.Kp << endl;
    cout << "Kd: " << endl << ctrl_param.Kd << endl;
    cout << "tilt_angle_max: " << ctrl_param.tilt_angle_max << endl;
}

void pos_controller_UDE::printf_result()
{
    cout << "UDE Controller Output:" << endl;
    cout << "Baseline Force: [" << u_l(0) << ", " << u_l(1) << ", " << u_l(2) << "]" << endl;
    cout << "Disturbance Force: [" << u_d(0) << ", " << u_d(1) << ", " << u_d(2) << "]" << endl;
    cout << "Total Desired Force: [" << F_des(0) << ", " << F_des(1) << ", " << F_des(2) << "]" << endl;
    cout << "Attitude Command: [" << u_att(0)*180/M_PI << ", " << u_att(1)*180/M_PI << ", " << u_att(2)*180/M_PI << ", " << u_att(3) << "]" << endl;
    cout << "Position Error: " << tracking_error.pos_error_norm << endl;
    cout << "Velocity Error: " << tracking_error.vel_error_norm << endl;
}

#endif