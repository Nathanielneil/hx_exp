#ifndef POS_CONTROLLER_ADRC_H
#define POS_CONTROLLER_ADRC_H

#include <ros/ros.h>
#include <math.h>
#include <Eigen/Eigen>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/ADRCState.h>
#include <iostream>
#include <algorithm>
#include <vector>
#include "math_utils.h"
#include "controller_utils.h"
#include "geometry_utils.h"
#include "printf_utils.h"
#include <Eigen/Dense>

//using namespace Eigen;
using namespace std;
 
class pos_controller_ADRC
{
    public:
        pos_controller_ADRC(){};

        void init(ros::NodeHandle& nh);//初始化，创建NodeHandle的实例

        //void set_initial_pos(const Eigen::Vector3d& pos);

        void set_desired_state(const Desired_State& des)//期望值
        {
            desired_state = des;
        }

        void set_current_state(const prometheus_msgs::UAVState& state)//实际值设置
        {
            uav_state = state;

            for(int i=0; i<3; i++)//位置和速度
            {
                current_state.pos(i) = uav_state.position[i];
                current_state.vel(i) = uav_state.velocity[i];
            }

            current_state.q.w() = uav_state.attitude_q.w; //四元数
            current_state.q.x() = uav_state.attitude_q.x;
            current_state.q.y() = uav_state.attitude_q.y;
            current_state.q.z() = uav_state.attitude_q.z; 

            current_state.yaw = geometry_utils::get_yaw_from_quaternion(current_state.q);//当前偏航角，geometry_utils.h中定义的函数
        }

        void printf_param(); //函数声明,打印参数和打印结果函数
        void printf_result();

        Eigen::Vector4d update(float controller_hz); //函数声明，返回Eigen::Vector4d 类型
        void set_control_command(const prometheus_msgs::UAVCommand& com)
        {
            uav_command = com; // 需要在uav_controller.c文件中调用，以获取指令信息，下两个函数同理
        }
        void set_control_state(const prometheus_msgs::UAVControlState& con_state)
        {
            uav_control_state = con_state; // 获取控制状态信息
        }

        ESO_Param ESO_x; // 三轴上的ESO参数与变量
        ESO_Param ESO_y;
        ESO_Param ESO_z;
        prometheus_msgs::ADRCState adrc_state_pub;


    private:
        Ctrl_Param_ADRC ctrl_param;//结构体Ctrl_Param_AMESO,定义实例 ctrl_param
        Desired_State desired_state;
        Current_State current_state;
        prometheus_msgs::UAVState uav_state; //创建类型为prometheus_msgs::UAVState的变量 uav_state
        Tracking_Error_Evaluation tracking_error; //创建结构体变量 traching_error
        prometheus_msgs::UAVCommand uav_command; // 从主函数中引入指令信息与控制状态
        prometheus_msgs::UAVCommand uav_command_last; 
        prometheus_msgs::UAVControlState uav_control_state; 
        prometheus_msgs::UAVControlState uav_control_state_last;

        //
        

        Eigen::Vector3d int_e_v;            // 积分
        
        float u;                            //输入u为总升力
        float un;                           //un为标称系统输入
        float uf;                           //uf为feedback
        float uc;  
        float hatd ;
        float uo;                           //uo为第一次拨杆的输入
        Eigen::Vector3d dhat;               //观测器

        Eigen::Vector3d F_des;              //创建变量F_des，期望姿态角
        Eigen::Vector4d u_att;              // 期望姿态角（rad）+期望油门（0-1）
        Eigen::Vector2d t_td;               //td参数
        float s;                            //滑模状态
        Eigen::Vector2d z;                  //滑模辅助状态
        Eigen::Vector2d xbeta;                  //滑模辅助状态
        Eigen::Vector2d TD;
        Eigen::Vector3d u_l,u_d;
        Eigen::Vector3d integral;
        //Eigen::VectorXd w(9);
        //std::vector<float> para_w[9]= vector<float> para_w(9);
        std::vector<float> para_w = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        //float para_w;
        //vector<float>para_w[9];
        bool con_init;
        int method_choose;
        float u_att_min; float u_att_max; float Thrust_des_min; float Thrust_des_max; // 归一化参数
        float Thr_alpha; float Thr_k1; float Thr_k2; float Thr_k3;
        Eigen::Vector3d u_v; // 位置控制器输出
        int flag;
        float hx;               //自适应基函数替代x2的变量
        Eigen::Vector3d xi_v; // 虚拟位置
};



//参数设置函数；从ros接收参数并储存
void pos_controller_ADRC::init(ros::NodeHandle& nh)//定义成员函数，接收一个ros::NodeHandle类型的引用参数nh
{

    // 【参数】控制参数，这部分需要修改
    //ctrl_param.amesogain_l.setZero();//setZero()函数将向量赋值为0
    //ctrl_param.amesogain_l.setZero();
    u=0;       //设置输入初值
    un=0;
    uc=0;
    uf=0;
    uo=0;
    hatd=0;
    hx=0;
    F_des.setZero();
    u_att.setZero();
    dhat.setZero();
    xi_v.setZero();
     u_l.setZero();
     u_d.setZero();
    s=0;
    z.setZero();
    integral.setZero();
    xbeta.setZero();
    TD.setZero();
    u_v.setZero();
    con_init = false;
  
    // 【参数】无人机质量
    nh.param<float>("ameso_gain/quad_mass" , ctrl_param.quad_mass, 1.0f);//从ros的参数服务器上获取pid_gain/quad_mass的参数值，并储存在ctrl_param.quad_mass中，无法获取则设置为1.0f
    // 【参数】悬停油门
    nh.param<float>("ameso_gain/hov_percent" , ctrl_param.hov_percent, 0.5f);
    // 【参数】XYZ积分上限
    nh.param<float>("ameso_gain/pxy_int_max"  , ctrl_param.int_max[0], 0.5f);
    nh.param<float>("ameso_gain/pxy_int_max"  , ctrl_param.int_max[1], 0.5f);
    nh.param<float>("ameso_gain/pz_int_max"   , ctrl_param.int_max[2], 0.5f);

    //nh.param<float>("ameso_gain/tilt_angle_max" , ctrl_param.tilt_angle_max, 10.0f);


    //
    nh.param<float>("ameso_gain/beta_max" , ctrl_param.beta_max, 0.5f);
    nh.param<float>("ameso_gain/C1" , ctrl_param.C[0], 0.5f);
    nh.param<float>("ameso_gain/C2" , ctrl_param.C[1], 0.5f);
    nh.param<float>("ameso_gain/sigmaD" , ctrl_param.sigma_D, 0.5f);
    nh.param<float>("ameso_gain/amesogain_l" , ctrl_param.amesogain_l, 0.5f);

    uav_command_last.Agent_CMD = prometheus_msgs::UAVCommand::Init_Pos_Hover;
    uav_command_last.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
    uav_command_last.position_ref = {0, 0, 0};
    uav_command_last.yaw_ref = 0.0;

   
    ctrl_param.g = 9.8; //重力加速度

    printf_param();//打印参数

    flag=0;
}

// 输入：
// 无人机位置、速度、偏航角
// 期望位置、速度、加速度、偏航角
// 输出：
// 期望姿态 + 期望油门

Eigen::Vector4d pos_controller_ADRC::update(float controller_hz)
{
/*
if (uav_control_state_last.control_state == prometheus_msgs::UAVControlState::COMMAND_CONTROL &&
        uav_control_state.control_state == prometheus_msgs::UAVControlState::RC_POS_CONTROL)
        {
            flag++;
        }*/

    float T=1/controller_hz;     //采样周期

    // 定义位置误差和速度误差
    Eigen::Vector3d pos_error = current_state.pos - desired_state.pos ;
    Eigen::Vector3d vel_error = current_state.vel - desired_state.vel ;
    
    tracking_error.input_error(pos_error,vel_error); //tracking_error为跟踪误差评估类的实例，将上面的位置和速度误差输入

    float ez = pos_error[2];   //z方向的跟踪误差和速度误差//仿真中的xi
    float ez2 = vel_error[2];

    //设置初值
    float ezn; //nominal系统状态；//仿真中的xin
    float ezn2;
    float ezn0;
    float ezn20;
    if(!con_init)
    {
     ezn=ez; //nominal系统状态；//仿真中的xin
     ezn2=ez2;
     
     ezn0=ez; //nominal系统状态；//仿真中的xin
     ezn20=ez2;

     con_init = true;
    }

    


    float beta1=1+std::min(ctrl_param.beta_max,powf(abs(ezn),0.2))*((abs(ezn)-1>0)?1:-1);//定义滑模面指数变量
    float beta2=1+std::min(ctrl_param.beta_max,powf(abs(ezn2),0.2))*((abs(ezn2)-1>0)?1:-1);
    
    xbeta[0]=powf(abs(ezn),beta1)*((ezn>0)?1:-1);
    xbeta[1]=powf(abs(ezn2),beta2)*((ezn2>0)?1:-1);

    s=ctrl_param.C[0]*ezn+ctrl_param.C[1]*ezn2+ctrl_param.C[0]*z[0]+ctrl_param.C[1]*z[1]-ctrl_param.C[0]*ezn0-ctrl_param.C[1]*ezn20;

    un=powf((ctrl_param.C[1]*T/ctrl_param.quad_mass),(-1))*(-0.8*s-ctrl_param.C[0]*(ezn+T*ezn2)-ctrl_param.C[1]*ezn2+ctrl_param.C[1]*T*ctrl_param.g+ctrl_param.C[1]
    *T*desired_state.acc[2]-ctrl_param.C[0]*z[0]-ctrl_param.C[1]*z[1]-ctrl_param.C[0]*T*xbeta[0]-ctrl_param.C[1]*T*xbeta[1]+ctrl_param.C[0]*ezn0+ctrl_param.C[1]*ezn20);


    ezn=ezn+T*ezn2;
    ezn2=ezn2+T/ctrl_param.quad_mass*un-T*ctrl_param.g-T*desired_state.acc[2];//标称系统迭代

    
    uf=-ctrl_param.quad_mass*(0.15*(ez-ezn)+3*(TD[1]-ezn2));

    hx=TD[1]+ desired_state.vel[2];

    std::vector<float> phi={1,sin(current_state.pos[2]),sin(hx),cos(current_state.pos[2]),cos(hx),sin(2*current_state.pos[2]),sin(2*hx),cos(2*current_state.pos[2]),cos(2*hx)};//基函数}

    float apd;
    apd=0;
    for (int j=0;j<9;j++)
    {
     apd=apd+para_w[j]*phi[j];
     //apd=apd+para_w*phi[j];
    }

    
    hatd =dhat[2]+apd;

    uc=-ctrl_param.quad_mass*hatd;

    ///////////////////////////////////////////////////////////////////
    //设计uo
    /////////////////////////////////////

   u=un+uc+uf;





 // 期望拉力

   //ESO
    float ee=ez-dhat[0];
    dhat[0] = dhat[0]+T*(dhat[1]+3*ctrl_param.amesogain_l*ee);
    dhat[1] = dhat[1]+T*(-ctrl_param.g+1/ctrl_param.quad_mass*u-desired_state.acc[2]+hatd+3*ctrl_param.amesogain_l*ctrl_param.amesogain_l*ee);
    dhat[2] = dhat[1]+T*(ctrl_param.amesogain_l*ctrl_param.amesogain_l*ctrl_param.amesogain_l*ee);
    //adptive model
    float en = TD[1]-ezn2;
   for(int j=0;j<9;j++)
   {
    para_w[j]=para_w[j]+T*0.8*(en*phi[j]-0.9*abs(en)*para_w[j]);
    //para_w=para_w+T*0.8*(en*phi[j]-0.9*abs(en)*para_w);
   }
    //TD
    TD[0]=TD[0]+T*TD[1];
    TD[1]=TD[1]-T*(1/(T*2*T)*(TD[0]-ez)+(T+2*T)/(T*2*T)*TD[1]);
    

    z=z+T*xbeta;

    // 期望加速度 = 期望加速度 + Kp * 位置误差 + Kv * 速度误差 + Kv * 积分项
    //Eigen::Vector3d des_acc = desired_state.acc + ctrl_param.Kp * pos_error + ctrl_param.Kv * vel_error + ctrl_param.Kvi* int_e_v;

	// 期望力 = 质量*控制量 + 重力抵消
    // F_des是基于模型的位置控制器计算得到的三轴期望推力（惯性系），量纲为牛
    // u_att是用于PX4的姿态控制输入，u_att 前三位是roll pitch yaw， 第四位为油门值[0-1]
    //F=ma+mg
    //F_des = des_acc * ctrl_param.quad_mass + ctrl_param.quad_mass * ctrl_param.g;



    double Thrust_des = u;
        //u_v[2]=u;//为什么等于u
        u_att(2) = desired_state.yaw;
        u_att(0) = asin((sin(u_att(2))*u_v[0]-cos(u_att(2))*u_v[1]) / (Thrust_des/ctrl_param.quad_mass));
        u_att(1) = atan((cos(u_att(2))*u_v[0]+sin(u_att(2))*u_v[1]) / (u_v[2]+ctrl_param.g));


 /*
 if (flag<0)
 {if (uav_control_state_last.control_state == prometheus_msgs::UAVControlState::RC_POS_CONTROL&&
        uav_control_state.control_state == prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {
    float max_pos_error = 3.0;
    float max_vel_error = 3.0;

    for (int i=0; i<3; i++)
    {
        if(abs(pos_error[i]) > max_pos_error)
        {            
            pos_error[i] = (pos_error[i] > 0) ? max_pos_error : -max_pos_error;
        }

        if(abs(vel_error[i]) > max_vel_error)
        {            
            vel_error[i] = (vel_error[i] > 0) ? max_vel_error : -max_vel_error;
        }

    }

    // UDE算法
    u_l = desired_state.acc + 0.5 * pos_error + 2 * vel_error;
    u_d = - 1.0 / 1 * (0.5 * integral + 2 * pos_error + vel_error);

    // 更新积分项
    for (int i=0; i<3; i++)
    {
        float int_start_error = 0.5;
        if(abs(pos_error[i]) < int_start_error)
        {
            integral[i] += pos_error[i] * T;
        }else
        {
            integral[i] = 0;
        }

        if(abs(u_d[i]) > ctrl_param.int_max[i])
        {
            PCOUT(2, YELLOW, "u_d saturation!");
            u_d[i] = (u_d[i] > 0) ? ctrl_param.int_max[i] : -ctrl_param.int_max[i];
        }
    }


   // 期望加速度
    Eigen::Vector3d u_j = u_l - u_d;
        
	// 期望力 = 质量*控制量 + 重力抵消

    Eigen::Vector3d G;
    G<< 0,0,ctrl_param.g;
	 Eigen::Vector3d F_des = u_j * ctrl_param.quad_mass + ctrl_param.quad_mass * G;
        
    
	// 如果向上推力小于重力的一半
	// 或者向上推力大于重力的两倍
	if (F_des(2) < 0.5 * ctrl_param.quad_mass * ctrl_param.g)
	{
		F_des = F_des / F_des(2) * (0.5 * ctrl_param.quad_mass * ctrl_param.g);
	}
	else if (F_des(2) > 2 * ctrl_param.quad_mass * ctrl_param.g)
	{
		F_des = F_des / F_des(2) * (2 * ctrl_param.quad_mass * ctrl_param.g);
	}

	// 角度限制幅度
	if (std::fabs(F_des(0)/F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))
	{
		PCOUT(2, YELLOW, "pitch too tilt");
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max));
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))
	{
		PCOUT(2, YELLOW, "roll too tilt");
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max));	
	}

    // F_des是位于ENU坐标系的,F_c是FLU
    Eigen::Matrix3d wRc = geometry_utils::rotz(current_state.yaw);
    Eigen::Vector3d F_c = wRc.transpose() * F_des;
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);


    // 期望roll, pitch
    u_att(0)  = std::atan2(-fy, fz);
    u_att(1)  = std::atan2( fx, fz);
    u_att(2)  = desired_state.yaw; 

 }
flag++;
 }
*/

///////////////////////////////////////////////////////////

ESO_x.z1=0;
ESO_x.z2=0;
ESO_x.z3=0;

ESO_y.z1=0;
ESO_y.z2=0;
ESO_y.z3=0;

ESO_z.z1=dhat[0];
ESO_z.z2=dhat[1];
ESO_z.z3=dhat[2];

    adrc_state_pub.ESO_state_x = {ESO_x.z1, ESO_x.z2, ESO_x.z3}; // ESO状态
    adrc_state_pub.ESO_state_y = {ESO_y.z1, ESO_y.z2, ESO_y.z3};
    adrc_state_pub.ESO_state_z = {ESO_z.z1, ESO_z.z2, ESO_z.z3};
    adrc_state_pub.tracking_error_body = {xi_v[0], xi_v[1], xi_v[2]}; // ESO输入变量
    adrc_state_pub.controller_output = {u_v[0], u_v[1], u_v[2]};
    adrc_state_pub.des_angle = {u_att(0), u_att(1), u_att(2)};
    adrc_state_pub.thrust = Thrust_des;
    uav_control_state_last = uav_control_state;
    uav_command_last = uav_command;


switch (method_choose)
    {
    case 1:
    {// 法一：过原点与点(hov_percent, mg)的线性关系（原方法）
        // 悬停油门与电机参数有关系,也取决于质量
        double full_thrust = ctrl_param.quad_mass * ctrl_param.g/ ctrl_param.hov_percent;
        // 油门 = 期望推力/最大推力
        // 这里相当于认为油门是线性的,满足某种比例关系,即认为某个重量 = 悬停油门
        u_att(3) = Thrust_des / full_thrust;
    break;
    }
    case 2:
    {// 法二：线性关系Thrust_des = Thr_k * u_att + Thr_b，过点1 (hov_percent, mg)与点2 (1, 2.5mg)，点的位置可更改
        float Thr_x1 = ctrl_param.hov_percent; double Thr_y1 = ctrl_param.quad_mass*ctrl_param.g; // 点1
        float Thr_x2 = 1.0f;                   double Thr_y2 = 2.5*ctrl_param.quad_mass*ctrl_param.g; // 点2
        // 直线斜率Thr_k计算
        double Thr_k = (Thr_y2 - Thr_y1)/(Thr_x2 - Thr_x1);
        // 将点(1, 2.5mg)代入直线中，求得Thr_b
        double Thr_b = Thr_y2 - Thr_k * Thr_x2;
        u_att(3) = (Thrust_des - Thr_b)/Thr_k;
    break;
    }
    case 3:
    {// 法三：二次函数关系Thrust_des = Thr_alpha * u_att^2 + (1-Thr_alpha) * u_att，变量归一化处理
        // 此处归一化处理的默认最大值为(1, 2.5mg)，可修改参数文件中u_att_max和Thrust_des_max的值加以调整
        // 待调参数Thr_alpha
        double Thrust_des_nor = (Thrust_des - Thrust_des_min)/(Thrust_des_max - Thrust_des_min);
        double u_att_nor = (-(1.0-Thr_alpha) + sqrt((1.0-Thr_alpha)*(1.0-Thr_alpha)+4.0*Thr_alpha*Thrust_des_nor))/(2.0*Thr_alpha);
        u_att(3) = u_att_nor*(u_att_max - u_att_min) + u_att_min;
    break;
    }
    case 4:
    {// 法四：二次函数关系Thrust_des = Thr_k3 *(Thr_k1 * u_att^2 + Thr_k2 * u_att)，变量归一化处理
        // 此处归一化处理的默认最大值为(1, 2.5mg)，可修改参数文件中u_att_max和Thrust_des_max的值加以调整
        // 待调参数Thr_k1，Thr_k2，Thr_k3
        double Thrust_des_nor = (Thrust_des - Thrust_des_min)/(Thrust_des_max - Thrust_des_min);
        double u_att_nor = (-Thr_k2 + sqrt(Thr_k2*Thr_k2 + 4.0*Thr_k1*Thrust_des_nor/Thr_k3))/(2.0*Thr_k1);
        u_att(3) = u_att_nor*(u_att_max - u_att_min) + u_att_min;
    break;
    }
    default:
    {
        double full_thrust = ctrl_param.quad_mass * ctrl_param.g / ctrl_param.hov_percent;
        u_att(3) = Thrust_des / full_thrust;
    }
    }

    if(u_att(3) < 0.3)
    {
        u_att(3) = 0.3;
        PCOUT(2, YELLOW, "throttle too low");
    }

    if(u_att(3) > 0.7)
    {
        u_att(3) = 0.7;
        PCOUT(2, YELLOW, "throttle too high");
    }

    return u_att;


/*
	// 角度限制幅度
	if (std::fabs(F_des(0)/F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))//两个方向的分量比值；std::fabs为绝对值函数，geometry_utils::toRad函数将角度从度转换为弧度
	{
        PCOUT(2, YELLOW, "pitch too tilt");//输出，俯仰角过大
		F_des(0) = F_des(0)/std::fabs(F_des(0)) * F_des(2) * std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max));
        //这行代码中，F_des(0)/std::fabs(F_des(0))=sgn(),设置为最大角度
	}

	// 角度限制幅度
	if (std::fabs(F_des(1)/F_des(2)) > std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max)))
	{
        PCOUT(2, YELLOW, "roll too tilt");
		F_des(1) = F_des(1)/std::fabs(F_des(1)) * F_des(2) * std::tan(geometry_utils::toRad(ctrl_param.tilt_angle_max));	
	}

    // F_des是位于ENU坐标系的,F_c是FLU
    Eigen::Matrix3d wRc = geometry_utils::rotz(current_state.yaw); //旋转矩阵，旋转角度为current_state.yaw；查看geometry_utils文件
    Eigen::Vector3d F_c = wRc.transpose() * F_des;  //坐标系转换，从ENU到FLU

    //定义三个分量
    double fx = F_c(0);
    double fy = F_c(1);
    double fz = F_c(2);

    // 期望roll, pitch
    u_att(0)  = std::atan2(-fy, fz);
    u_att(1)  = std::atan2( fx, fz);
    u_att(2)  = desired_state.yaw;

    // 无人机姿态的矩阵形式
    Eigen::Matrix3d wRb_odom = current_state.q.toRotationMatrix();//toRotationMatrix()表示四元数转化为等效3*3旋转矩阵的函数

    // 第三列
    Eigen::Vector3d z_b_curr = wRb_odom.col(2);

    // 机体系下的电机推力 相当于Rb * F_enu 惯性系到机体系
    double u1 = F_des.dot(z_b_curr); //.dot表示计算点乘

    // 悬停油门与电机参数有关系,也取决于质量
    double full_thrust = ctrl_param.quad_mass * ctrl_param.g(2) / ctrl_param.hov_percent; //T=mg/hov_percent

    // 油门 = 期望推力/最大推力
    // 这里相当于认为油门是线性的,满足某种比例关系,即认为某个重量 = 悬停油门
    //u_att(3)的取值，限幅
    u_att(3) = u1 / full_thrust;
    if(u_att(3) < 0.1)
    {
        u_att(3) = 0.1;
        PCOUT(2, YELLOW, "throttle too low");
    }

    if(u_att(3) > 1.0)
    {
        u_att(3) = 1.0;
        PCOUT(2, YELLOW, "throttle too high");
    }

    return u_att;
    */

}


//定义打印函数
void pos_controller_ADRC::printf_result()
{
    //固定的浮点显示
    cout.setf(ios::fixed);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);
    cout<<setprecision(2);
    cout << BLUE << "----> PID Position Controller Debug Info      : " << TAIL << endl;
    cout << BLUE << "----> pos_des         : " << desired_state.pos(0) << " [ m ] " << desired_state.pos(1) << " [ m ] " << desired_state.pos(2) << " [ m ] "<< TAIL << endl;
    cout << BLUE << "----> vel_des         : " << desired_state.vel(0) << " [ m ] " << desired_state.vel(1) << " [ m ] " << desired_state.vel(2) << " [ m ] "<< TAIL << endl;
    cout << BLUE << "----> acc_des         : " << desired_state.acc(0) << " [ m ] " << desired_state.acc(1) << " [ m ] " << desired_state.acc(2) << " [ m ] "<< TAIL << endl;
    cout << BLUE << "----> pos_now         : " << current_state.pos(0) << " [ m ] " << current_state.pos(1) << " [ m ] " << current_state.pos(2) << " [ m ] "<< TAIL << endl;
    cout << BLUE << "----> vel_now         : " << current_state.vel(0) << " [ m ] " << current_state.vel(1) << " [ m ] " << current_state.vel(2) << " [ m ] "<< TAIL << endl;
   

    cout << BLUE << "----> int_e_v         : " << int_e_v(0) << " [N] "<< int_e_v(1) << " [N] "<< int_e_v(2) << " [N] "<< TAIL << endl;
    
    cout << BLUE << "----> F_des           : " << F_des(0) << " [N] "<< F_des(1) << " [N] "<< F_des(2) << " [N] "<< TAIL << endl;
    
    cout << BLUE << "----> u_attitude      : " << u_att(0)*180/3.14 << " [deg] "<< u_att(1)*180/3.14 << " [deg] "<< u_att(2)*180/3.14 << " [deg] "<< TAIL << endl;
    cout << BLUE << "----> u_throttle      : " << u_att(3) << " [0-1] "<< TAIL << endl;
    cout << BLUE << "----> pos_error_mean  : " << tracking_error.pos_error_mean <<" [m] "<< TAIL <<endl;
    cout << BLUE << "----> vel_error_mean  : " << tracking_error.vel_error_mean <<" [m/s] "<< TAIL <<endl;
}

// 【打印参数函数】

void pos_controller_ADRC::printf_param()
{
    cout << GREEN << ">>>>>>>>>>>>>>>>>>>>>>>>>>ADRC Parameter <<<<<<<<<<<<<<<<<<<<<<<<<" << TAIL <<endl;
    cout << GREEN <<  "ctrl_param.quad_mass     : "<< ctrl_param.quad_mass<< TAIL <<endl;
    cout << GREEN <<  "ctrl_param.hov_percent   : "<< ctrl_param.hov_percent<< TAIL <<endl;
    
    //cout << GREEN <<  "ctrl_param.quad_mass     : "<< ctrl_param.quad_mass<< TAIL <<endl;
/*    cout << GREEN <<  "ctrl_param.hov_percent   : "<< ctrl_param.hov_percent<< TAIL <<endl;
    cout << GREEN <<  "pxy_int_max              : "<< ctrl_param.int_max[0]<< TAIL <<endl;
    cout << GREEN <<  "pz_int_max               : "<< ctrl_param.int_max[2]<< TAIL <<endl;

    cout << GREEN <<  "Kp_xy         : "<< ctrl_param.Kp(0,0) << TAIL <<endl;
    cout << GREEN <<  "Kp_z          : "<< ctrl_param.Kp(2,2) << TAIL <<endl;
    cout << GREEN <<  "Kv_xy         : "<< ctrl_param.Kv(0,0) << TAIL <<endl;
    cout << GREEN <<  "Kv_z          : "<< ctrl_param.Kv(2,2) << TAIL <<endl;
    cout << GREEN <<  "Kvi_xy        : "<< ctrl_param.Kvi(0,0) << TAIL <<endl;
    cout << GREEN <<  "Kvi_z         : "<< ctrl_param.Kvi(2,2) << TAIL <<endl;
    cout << GREEN <<  "Ka_xy         : "<< ctrl_param.Ka(0,0) << TAIL <<endl;
    cout << GREEN <<  "Ka_z          : "<< ctrl_param.Ka(2,2) << TAIL <<endl;
    cout << GREEN <<  "tilt_angle_max: "<< ctrl_param.tilt_angle_max << TAIL <<endl;
*/

}
#endif