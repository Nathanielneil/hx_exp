#ifndef __HX_CONTROLLER_UTILS_H__
#define __HX_CONTROLLER_UTILS_H__

#include <Eigen/Eigen>
#include <math.h>
#include <numeric>

using namespace std;

// Desired state structure
struct Desired_State
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Quaterniond q;
    double yaw;
};

// Current state structure
struct Current_State
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond q;
    double yaw;
};

// ADRC controller parameters
struct Ctrl_Param_ADRC
{
    float quad_mass;              // UAV mass
    float tilt_angle_max;         // Maximum tilt angle (degrees)
    float hov_percent;            // Hover throttle percentage
    float g;
    Eigen::Vector3f int_max;
    float amesogain_l;
    float beta_max;
    Eigen::Vector2f C;
    float sigma_D;
};

// PID controller parameters
struct Ctrl_Param_PID
{
    float quad_mass;        
    float tilt_angle_max;
    float hov_percent;
    Eigen::Vector3d g;
    Eigen::Vector3f int_max;
    Eigen::Matrix3d Kp;
    Eigen::Matrix3d Kv;
    Eigen::Matrix3d Kvi;
    Eigen::Matrix3d Ka;
};

// UDE controller parameters
struct Ctrl_Param_UDE
{
    double T_ude;
    float tilt_angle_max;
    float quad_mass;
    float hov_percent;
    Eigen::Vector3d g;
    Eigen::Vector3f int_max;
    Eigen::Matrix3d Kp;
    Eigen::Matrix3d Kd;
};

// Extended State Observer parameters
struct ESO_Param
{
    double z1;      // Position estimation
    double z2;      // Velocity estimation  
    double z3;      // Total disturbance estimation
    double x1;      // Input variable (position signal)
    double u1;      // Controller signal
    double error;   // Error between input and position estimation
    float l1;       // ESO parameters
    float l2;
    float l3;
    float eps;      // Epsilon
    float ell1;     // L parameters
    float ell2;
    float ell3;
    float l0;       // L0
    float alpha;    // Alpha
};

// Saturation controller parameters
struct Sat_Con_param
{
    double y1h;     // Controller variables
    double y2h;
    double u;       // Controller output
    float k1;       // Controller parameters
    float k2;
    float L1;       // Saturation function parameters
    float M1;
    float L2;
    float M2;
};

// Tracking error evaluation class
class Tracking_Error_Evaluation
{
    public:
        Tracking_Error_Evaluation(){
            pos_error_norm = 0.0;
            vel_error_norm = 0.0;
        };

        std::vector<double> pos_error_vector;
        std::vector<double> vel_error_vector;

        double pos_error_mean{0};
        double vel_error_mean{0};
        double pos_error_norm{0};
        double vel_error_norm{0};
        
        Eigen::Vector3d pos_error;
        Eigen::Vector3d vel_error;

        void input_error(Eigen::Vector3d position_error, Eigen::Vector3d velocity_error)
        {
            pos_error = position_error;
            vel_error = velocity_error;
            
            double track_error_pos = position_error.norm();
            double track_error_vel = velocity_error.norm();
            
            pos_error_norm = track_error_pos;
            vel_error_norm = track_error_vel;
            
            pos_error_vector.insert(pos_error_vector.begin(), track_error_pos);
            vel_error_vector.insert(vel_error_vector.begin(), track_error_vel);
            
            if (pos_error_vector.size() > Slide_window)
            {
                pos_error_vector.pop_back();
            }
            if (vel_error_vector.size() > Slide_window)
            {
                vel_error_vector.pop_back();
            }
            
            // Calculate moving average
            vel_error_mean = std::accumulate(vel_error_vector.begin(), vel_error_vector.end(), 0.0) / vel_error_vector.size();
            pos_error_mean = std::accumulate(pos_error_vector.begin(), pos_error_vector.end(), 0.0) / pos_error_vector.size();
        }

    private:
        int Slide_window = 100;
};

namespace controller_utils 
{
    // Controller utility functions can be added here
}

#endif