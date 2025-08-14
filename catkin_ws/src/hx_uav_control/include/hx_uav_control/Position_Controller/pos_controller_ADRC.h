#ifndef HX_POS_CONTROLLER_ADRC_H
#define HX_POS_CONTROLLER_ADRC_H

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Eigen>
#include <hx_msgs/UAVState.h>

#include "geometry_utils.h"
#include "controller_utils.h"
#include "printf_utils.h"

using namespace std;

class pos_controller_ADRC
{
    public:
        pos_controller_ADRC(){};

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
        Ctrl_Param_ADRC ctrl_param;
        Desired_State desired_state;
        Current_State current_state;
        hx_msgs::UAVState uav_state;

        Tracking_Error_Evaluation tracking_error;

        // ADRC specific components
        ESO_Param eso_x, eso_y, eso_z;          // Extended State Observers for X, Y, Z axes
        Sat_Con_param controller_x, controller_y, controller_z;  // Saturation controllers
        
        Eigen::Vector3d F_des;
        Eigen::Vector4d u_att;                   // Desired attitude [roll, pitch, yaw, throttle]
        
        // ADRC functions
        double fal(double e, double alpha, double delta);
        double fhan(double x1, double x2, double r, double h);
        void update_eso(ESO_Param& eso, double y, double u, double dt);
        double update_saturation_controller(Sat_Con_param& controller, double z1, double z2, double z3, double v, double dt);
};

void pos_controller_ADRC::init(ros::NodeHandle& nh)
{
    // Physical parameters
    nh.param<float>("adrc_gain/quad_mass", ctrl_param.quad_mass, 1.0f);
    nh.param<float>("adrc_gain/hov_percent", ctrl_param.hov_percent, 0.5f);
    nh.param<float>("adrc_gain/tilt_angle_max", ctrl_param.tilt_angle_max, 10.0f);
    
    ctrl_param.g = 9.8f;
    
    // ESO parameters for X axis
    nh.param<float>("adrc_gain/eso_l1", eso_x.l1, 100.0f);
    nh.param<float>("adrc_gain/eso_l2", eso_x.l2, 200.0f);
    nh.param<float>("adrc_gain/eso_l3", eso_x.l3, 100.0f);
    nh.param<float>("adrc_gain/eso_alpha", eso_x.alpha, 0.5f);
    nh.param<float>("adrc_gain/eso_eps", eso_x.eps, 0.01f);
    
    // Initialize ESO states
    eso_x.z1 = eso_x.z2 = eso_x.z3 = 0.0;
    eso_y = eso_x;  // Copy parameters
    eso_z = eso_x;
    
    // Controller parameters
    nh.param<float>("adrc_gain/controller_k1", controller_x.k1, 10.0f);
    nh.param<float>("adrc_gain/controller_k2", controller_x.k2, 5.0f);
    nh.param<float>("adrc_gain/controller_L1", controller_x.L1, 1.0f);
    nh.param<float>("adrc_gain/controller_M1", controller_x.M1, 0.5f);
    nh.param<float>("adrc_gain/controller_L2", controller_x.L2, 1.0f);
    nh.param<float>("adrc_gain/controller_M2", controller_x.M2, 0.5f);
    
    // Initialize controller states
    controller_x.y1h = controller_x.y2h = controller_x.u = 0.0;
    controller_y = controller_x;  // Copy parameters
    controller_z = controller_x;

    printf_param();
}

double pos_controller_ADRC::fal(double e, double alpha, double delta)
{
    if (abs(e) <= delta)
    {
        return e / pow(delta, 1 - alpha);
    }
    else
    {
        return math_utils::sign_function(e) * pow(abs(e), alpha);
    }
}

double pos_controller_ADRC::fhan(double x1, double x2, double r, double h)
{
    double d = r * h * h;
    double a0 = h * x2;
    double y = x1 + a0;
    double a1 = sqrt(d * (d + 8.0 * abs(y)));
    double a2 = a0 + math_utils::sign_function(y) * (a1 - d) / 2.0;
    double sy = (math_utils::sign_function(y + d) - math_utils::sign_function(y - d)) / 2.0;
    double a = (a0 + y - a2) * sy + a2;
    double sa = (math_utils::sign_function(a + d) - math_utils::sign_function(a - d)) / 2.0;
    return -r * (a / d - math_utils::sign_function(a)) * sa - r * math_utils::sign_function(a);
}

void pos_controller_ADRC::update_eso(ESO_Param& eso, double y, double u, double dt)
{
    eso.error = eso.z1 - y;
    eso.z1 = eso.z1 + dt * (eso.z2 - eso.l1 * eso.error);
    eso.z2 = eso.z2 + dt * (eso.z3 - eso.l2 * fal(eso.error, 0.5, eso.eps) + u);
    eso.z3 = eso.z3 + dt * (-eso.l3 * fal(eso.error, 0.25, eso.eps));
}

double pos_controller_ADRC::update_saturation_controller(Sat_Con_param& controller, double z1, double z2, double z3, double v, double dt)
{
    double e1 = v - z1;
    double e2 = 0 - z2;  // Desired velocity is 0 for position control
    
    // Saturation functions
    double sat1 = (abs(e1) <= controller.L1) ? e1 / controller.M1 : math_utils::sign_function(e1);
    double sat2 = (abs(e2) <= controller.L2) ? e2 / controller.M2 : math_utils::sign_function(e2);
    
    controller.y1h = controller.y1h + dt * controller.y2h;
    controller.y2h = controller.y2h + dt * (controller.k1 * sat1 + controller.k2 * sat2);
    
    // Control output compensating for disturbance
    controller.u = controller.k1 * sat1 + controller.k2 * sat2 - z3;
    
    return controller.u;
}

Eigen::Vector4d pos_controller_ADRC::update(float controller_hz)
{
    float dt = 1.0 / controller_hz;
    
    // Update ESO for each axis
    update_eso(eso_x, current_state.pos(0), controller_x.u, dt);
    update_eso(eso_y, current_state.pos(1), controller_y.u, dt);
    update_eso(eso_z, current_state.pos(2), controller_z.u, dt);
    
    // Update controllers for each axis
    double u_x = update_saturation_controller(controller_x, eso_x.z1, eso_x.z2, eso_x.z3, desired_state.pos(0), dt);
    double u_y = update_saturation_controller(controller_y, eso_y.z1, eso_y.z2, eso_y.z3, desired_state.pos(1), dt);
    double u_z = update_saturation_controller(controller_z, eso_z.z1, eso_z.z2, eso_z.z3, desired_state.pos(2), dt);
    
    // Compose desired force vector
    F_des << u_x, u_y, u_z + ctrl_param.quad_mass * ctrl_param.g;
    
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
    Eigen::Vector3d e_pos = desired_state.pos - current_state.pos;
    Eigen::Vector3d e_vel = desired_state.vel - current_state.vel;
    tracking_error.input_error(e_pos, e_vel);
    
    return u_att;
}

void pos_controller_ADRC::printf_param()
{
    cout << "ADRC Controller Parameters:" << endl;
    cout << "quad_mass: " << ctrl_param.quad_mass << endl;
    cout << "hov_percent: " << ctrl_param.hov_percent << endl;
    cout << "ESO l1: " << eso_x.l1 << ", l2: " << eso_x.l2 << ", l3: " << eso_x.l3 << endl;
    cout << "Controller k1: " << controller_x.k1 << ", k2: " << controller_x.k2 << endl;
    cout << "tilt_angle_max: " << ctrl_param.tilt_angle_max << endl;
}

void pos_controller_ADRC::printf_result()
{
    cout << "ADRC Controller Output:" << endl;
    cout << "ESO States X: [" << eso_x.z1 << ", " << eso_x.z2 << ", " << eso_x.z3 << "]" << endl;
    cout << "ESO States Y: [" << eso_y.z1 << ", " << eso_y.z2 << ", " << eso_y.z3 << "]" << endl;
    cout << "ESO States Z: [" << eso_z.z1 << ", " << eso_z.z2 << ", " << eso_z.z3 << "]" << endl;
    cout << "Desired Force: [" << F_des(0) << ", " << F_des(1) << ", " << F_des(2) << "]" << endl;
    cout << "Attitude Command: [" << u_att(0)*180/M_PI << ", " << u_att(1)*180/M_PI << ", " << u_att(2)*180/M_PI << ", " << u_att(3) << "]" << endl;
    cout << "Position Error: " << tracking_error.pos_error_norm << endl;
    cout << "Velocity Error: " << tracking_error.vel_error_norm << endl;
}

#endif