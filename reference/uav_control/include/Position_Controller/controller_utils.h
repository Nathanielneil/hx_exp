#ifndef __CONTROLLER_UTILS_H__
#define __CONTROLLER_UTILS_H__

#include <Eigen/Eigen>
#include <math.h>
#include <numeric>

using namespace std;


//期望状态类，Eigen为向量头文件。下面定义3维度向量和四元数
struct Desired_State
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::Quaterniond q;
    double yaw;
};

//当前状态类
struct Current_State
{
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Quaterniond q;
    double yaw;
};


//AMESO参数
struct Ctrl_Param_ADRC
{
    float quad_mass;//机身质量
    float tilt_angle_max; //最大角度；此处输入为角度值
    float hov_percent;//悬停油门
    float g;
    Eigen::Vector3f int_max;
    float amesogain_l;
    float beta_max;
    Eigen::Vector2f C;
    float sigma_D;
};



//PID参数
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

//UDE参数
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

//NE参数
/*
struct Ctrl_Param_NE
{
    Eigen::Matrix3d Kp;
    Eigen::Matrix3d Kd;
    double T_ude;
    double T_ne;
    float tilt_angle_max;
    float quad_mass;
    float hov_percent;
    Eigen::Vector3d g;
    Eigen::Vector3f int_max;
};*/

struct ESO_Param
{
    double z1; // 位置估计值
    double z2; // 速度估计值
    double z3; // 总扰动估计值
    double x1; // 输入变量（位置信号）
    double u1; // 控制器信号
    double error; // 输入变量与位置估计值的误差
    float l1; // 参数，对应文章里的$l_1$, $l_2$, $l_3$
    float l2;
    float l3;
    float eps; // $\varepsilon$
    float ell1; // $\ell_1$, $\ell_2$, $\ell_3$
    float ell2;
    float ell3;
    float l0; // $l_0$
    float alpha; // $\alpha$
};

struct Sat_Con_param
{
    double y1h; // 控制器变量$\hat{y}_1$, $\hat{y}_2$
    double y2h;
    double u; // 控制器输出
    float k1; // 控制器参数
    float k2;
    float L1; // 饱和函数参数
    float M1;
    float L2;
    float M2;
};


// 追踪误差评估
class Tracking_Error_Evaluation
{
    public:
        Tracking_Error_Evaluation(){};

        std::vector<double> pos_error_vector;  //声明std::vector的变量实例，double型位置跟踪误差
        std::vector<double> vel_error_vector;  //速度跟踪误差

        double pos_error_mean{0}; //初始化位置误差
        double vel_error_mean{0};  //初始化速度误差

        void input_error(Eigen::Vector3d pos_error,Eigen::Vector3d vel_error)
        {
            double track_error_pos = pos_error.norm(); //位置误差范数赋值
            double track_error_vel = vel_error.norm(); //速度误差范数赋值
            pos_error_vector.insert(pos_error_vector.begin(), track_error_pos);//在开头插入track_error_pos的值
            vel_error_vector.insert(vel_error_vector.begin(), track_error_vel);
            //.size为std::vector中元素数量，使用.pop_back()保证元素个数不变
            if (pos_error_vector.size() > Slide_windouw)
            {
                pos_error_vector.pop_back();//去掉最后一个值
            }
            if (vel_error_vector.size() > Slide_windouw)
            {
                vel_error_vector.pop_back();
            }
            //计算迭代的平均值
            vel_error_mean = std::accumulate(vel_error_vector.begin(), vel_error_vector.end(), 0.0) / pos_error_vector.size();
            pos_error_mean = std::accumulate(pos_error_vector.begin(), pos_error_vector.end(), 0.0) / pos_error_vector.size();
        }

    private:

        int Slide_windouw=100;
};


namespace controller_utils 
{



}
#endif
