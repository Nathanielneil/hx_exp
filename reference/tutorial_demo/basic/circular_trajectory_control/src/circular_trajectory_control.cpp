/******************************************************************************
*例程简介: 讲解如何调用uav_control的接口实现无人机ENU坐标系下的XY速度Z位置控制以及
*         轨迹控制
*
*效果说明: 无人机起飞后开始按照圆的轨迹飞行,飞行结束后悬停30秒,然后降落
*
*备注:该例程仅支持Prometheus仿真,真机测试需要熟练掌握相关接口的定义后以及真机适配修改后使用
******************************************************************************/


#include <ros/ros.h>
#include <rosbag/bag.h>
#include <prometheus_msgs/UAVCommand.h>
#include <prometheus_msgs/UAVState.h>
#include <prometheus_msgs/UAVControlState.h>
#include <unistd.h>
#include <math.h>
#include <Eigen/Eigen>
#include "printf_utils.h"

#include <stdlib.h>

using namespace std;

const float PI = 3.1415926;

//创建无人机相关数据变量(全局变量)
prometheus_msgs::UAVCommand uav_command;
prometheus_msgs::UAVState uav_state;
prometheus_msgs::UAVControlState uav_control_state;
prometheus_msgs::UAVControlState uav_control_state_last;
//创建圆形跟踪的相关变量
//整个圆形的飞行时间
float circular_time;
//每次控制数据更新时的弧度增量
float angle_increment;
//无人机的合速度也就是圆的线速度
float velocity;
//无人机的控制频率，控制周期
float control_rate;
float dt;
//圆的半径
float radius;

//通过设定整个圆的飞行时间,控制频率,圆的半径来获取相关参数
void get_circular_property(float time, int rate, float radius)
{
    //计算角速度(rad/s)
    float w = 2*PI/time;
    //计算线速度(m/s)
    velocity = radius * w;
    //计算控制数据发布的弧度增量
    angle_increment = w/rate;
}//因为是全局变量，所以没有返回值



//无人机状态回调函数，当一个新的UAV状态消息到达时，会调用该函数，将新的状态信息存储到uav_state变量中。
void uav_state_cb(const prometheus_msgs::UAVState::ConstPtr &msg)
{
    uav_state = *msg;
}

//无人机控制状态回调函数
void uav_control_state_cb(const prometheus_msgs::UAVControlState::ConstPtr &msg)
{
    uav_control_state = *msg;
}

//主函数
int main(int argc, char** argv)//argc 和 argv分别代表参数数量和参数列表
{
    //ROS初始化,设定节点名
    ros::init(argc , argv, "body_xyz_pos_control");
    //创建句柄
    ros::NodeHandle n;
    //获取无人机id,起飞高度
    int uav_id;
    float takeoff_height;
    ros::param::get("~uav_id", uav_id);
    ros::param::get("/uav_control_main_1/control/Takeoff_height", takeoff_height);
    
    //创建无人机控制命令发布者
    ros::Publisher uav_command_pub = n.advertise<prometheus_msgs::UAVCommand>("/uav" + std::to_string(uav_id) + "/prometheus/command", 10);
    //创建无人机状态命令订阅者
    ros::Subscriber uav_state_sub = n.subscribe<prometheus_msgs::UAVState>("/uav" + std::to_string(uav_id) + "/prometheus/state", 10, uav_state_cb);
    //创建无人机控制状态命令订阅者
    ros::Subscriber uav_control_state_sub = n.subscribe<prometheus_msgs::UAVControlState>("/uav" + std::to_string(uav_id) + "/prometheus/control_state", 10, uav_control_state_cb);
    //循环频率设置为1HZ
    ros::Rate r(1);
    //创建命令发布标志位,命令发布则为true;初始化为false
    bool cmd_pub_flag = false;

    //固定的浮点显示
    cout.setf(ios::fixed);
    // setprecision(n) 设显示小数精度为2位
    cout << setprecision(2);
    //左对齐
    cout.setf(ios::left);
    // 强制显示小数点
    cout.setf(ios::showpoint);
    // 强制显示符号
    cout.setf(ios::showpos);
    
    //打印demo相关信息
    cout << GREEN << " [circular trajectory control] tutorial_demo start " << TAIL << endl;
    sleep(1);
    cout << GREEN << " Level: [Basic] " << TAIL << endl;
    sleep(1);
    cout << GREEN << " Please use the RC SWA to armed, and the SWB to switch the drone to [COMMAND_CONTROL] mode  " << TAIL << endl;

    //设定飞行时间为50S,控制频率为20HZ,半径为1米
    circular_time = 50;
    control_rate = 50;
    dt = 1/control_rate;
    radius = 0;

    get_circular_property(circular_time, control_rate, radius);

    bool circular_success = false;
    bool height_control_flag = false;
    int count = 0;
    int point_count = 0;
    int point_time = 10;
    int point_num = 0;
    int count_z = 0;
    float tracking_time_2 = 10;
    ros::Rate rate(control_rate);
    // 轨迹起始点
    Eigen::Vector3d target_pos = {0, 0, takeoff_height};

    // 创建ROSbag对象，定义路径
    rosbag::Bag bag;
    string file_path = "/bagfiles/Trajectory_Tracking.bag";
    string path = (getenv("HOME")+file_path).c_str();
    // bag.open("~/bagfiles/Trajectory_Tracking.bag", rosbag::bagmode::Write);
    rosbag::Bag bag_point;
    string file_path_point = "/bagfiles/Point_Tracking.bag";
    string path_point = (getenv("HOME")+file_path_point).c_str();
    // bag_point.open("~/bagfiles/Point_Tracking.bag", rosbag::bagmode::Write);
    bool bag_open_flag = false; // 打开bag文件标志位

int flag=0;
while(ros::ok())
    {   //确定无人机状态
        ros::spinOnce();
        //检测无人机是否处于[COMMAND_CONTROL]模式
        if (uav_control_state.control_state == prometheus_msgs::UAVControlState::COMMAND_CONTROL)
        {   
            // 是否第一次执行COMMAND_CONTROL指令
            if(!cmd_pub_flag)
                flag=3;    
        }
        else 
            flag=2;    

        // 检测控制模式从COMMAND_CONTROL切换到RC_POS_CONTROL的状态，以重置变量值
        if (uav_control_state_last.control_state == prometheus_msgs::UAVControlState::COMMAND_CONTROL &&
        uav_control_state.control_state == prometheus_msgs::UAVControlState::RC_POS_CONTROL)
        {
            bag_open_flag = false;
            if(point_count <= control_rate*point_time)
                bag_point.close();
            if(circular_success == false)
                bag.close();
            count = 0;
            point_count = 0;
            point_num++;
            cmd_pub_flag = false;
            circular_success = false;
            //时间戳
            uav_command.header.stamp = ros::Time::now();
            //坐标系
            uav_command.header.frame_id = "ENU";
            //悬停
            uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
            //发布的命令ID加1
            uav_command.Command_ID += 1;
            //发布命令
            uav_command_pub.publish(uav_command);
            cout << YELLOW << " Parameters reset " << TAIL << endl;
        }
        uav_control_state_last = uav_control_state;

        //进入状态选择,并执行相应操作
        switch (flag)
        {
        
            case 2://无人机没有处于[COMMAND_CONTROL]模式
            {
                //在控制命令发布后,但无人机未结束任务的情况下,此时无人机未处于[COMMAND_CONTROL]控制状态,认为无人机出现意外情况,任务中止
                if(cmd_pub_flag)
                {
                    cout << RED << " Unknown error! [ENU XYZ position control] tutorial_demo aborted" << TAIL << endl;
                    r.sleep();
                    continue;
                }
                //命令未发布,等待无人机进入[COMMAND_CONTROL]状态
                else
                {
                    cout << YELLOW << " Wait for UAV to enter [COMMAND_CONTROL] MODE " << TAIL << endl;
                    r.sleep();
                    continue;
                }
            break;
            }

            case 3://第一次执行[COMMAND_CONTROL]模式
            {
                if(bag_open_flag == false)
                {
                    // 打开轨迹追踪与定点追踪的bag文件
                    bag.open(path, rosbag::bagmode::Write);
                    bag_point.open(path_point, rosbag::bagmode::Write);
                    // 标志位置true，即此函数只运行一次
                    bag_open_flag = true;
                }
                
                //检测无人机是否到达起飞高度
                if(!height_control_flag)
                {
                if(fabs(uav_state.position[2] - takeoff_height) >= 0.1 || count_z < tracking_time_2*control_rate)
                {
                    uav_command.position_ref[2] = takeoff_height;
                    count_z++;
                    rate.sleep();
                    continue;
                }
                cout << GREEN << " UAV takeoff success, wait for 10 seconds" << TAIL << endl;
                height_control_flag = true;
                sleep(2); // 等10秒再执行下面的程序
                }
                
                //时间戳
                uav_command.header.stamp = ros::Time::now();
                //坐标系
                uav_command.header.frame_id = "ENU";
                //Move模式
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                //Move_mode
                uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                //无人机将会以当前位置移动
                if(point_num==2)
                {// +x, -y
                    //uav_command.position_ref[2] = takeoff_height;
                    //uav_command.yaw_ref = 0.0;
                    uav_command.position_ref[0] = target_pos[0];
                    uav_command.position_ref[1] = target_pos[1];
                    uav_command.position_ref[2] = target_pos[2];
                }
                //else
                //{
                //}
                uav_command.yaw_ref = 0.0;
                //uav_command.position_ref[2] = 1.5;
                //发布的命令ID加1
                uav_command.Command_ID += 1;
                //发布命令
                uav_command_pub.publish(uav_command);
                //命令发布标志位置为true
                cmd_pub_flag = true;
                //打印无人机控制命令发布信息
                cout << GREEN << " Go to initial point " << TAIL << endl;
                //检测无人机是否到达初始点
                //bool initial_point_success = false;
                flag=4;
            break;
            }
            
            case 4://定点追踪状态
            {
                // ros::spinOnce();
                // 无人机定点追踪时，将command话题和state话题写入bag文件
                bag_point.write("/uav1/prometheus/command", ros::Time::now(), uav_command);
                bag_point.write("/uav1/prometheus/state", ros::Time::now(), uav_state);
                //当无人机距离目标位置±0.1米范围内且超过10秒时认为到达初始点
                Eigen::Vector3d uav_pos = {uav_state.position[0], uav_state.position[1], uav_state.position[2]};
                float distance = (uav_pos - target_pos).norm();
                if(distance < 0.1 && point_count >= control_rate*point_time) // 0.1
                {
                    cout << GREEN << " UAV arrived at initial point " << TAIL << endl;
                    //initial_point_success = true;
                    flag=5;
                    // 关闭bag文件
                    bag_point.close();

                }
                point_count++; //计数器
                rate.sleep(); // 频率20Hz
                // r.sleep(); // 频率1Hz
            break;
            }

            case 5://轨迹追踪状态
            {
                //时间戳
                uav_command.header.stamp = ros::Time::now();
                //坐标系
                uav_command.header.frame_id = "ENU";
                //Move模式
                uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Move;
                //Move_mode
                //非PX4_ORIGIN控制器时，仅支持XYZ_POS和TRAJECTORY两种子模式，其他的均不支持，需要修改！
                uav_command.Move_mode = prometheus_msgs::UAVCommand::TRAJECTORY;
                //无人机按照圆形轨迹飞行
                /*
                uav_command.position_ref[0] = cos(PI/10*count*dt); // count为计数变量，dt为时间间隔
                uav_command.position_ref[1] = sin(PI/10*count*dt);
                uav_command.position_ref[2] = target_pos[2];
                uav_command.velocity_ref[0] = -velocity*sin(count*angle_increment); // velocity=PI/10; angle_increment=PI/10*dt
                uav_command.velocity_ref[1] = velocity*cos(count*angle_increment);
                uav_command.velocity_ref[2] = 0;
                uav_command.acceleration_ref[0] = -PI*PI/100*cos(PI/10*count*dt);
                uav_command.acceleration_ref[1] = -PI*PI/100*sin(PI/10*count*dt);
                uav_command.acceleration_ref[2] = 0;
                uav_command.yaw_ref = 0;*/

                //上下飞
                ///*
                uav_command.position_ref[0] = 0; // count为计数变量，dt为时间间隔
                uav_command.position_ref[1] = 0;
                uav_command.position_ref[2] = 0.7+0.22*sin(dt*count)-0.2*cos(0.5*dt*count);
                uav_command.velocity_ref[0] = 0; // velocity=PI/10; angle_increment=PI/10*dt
                uav_command.velocity_ref[1] = 0;
                uav_command.velocity_ref[2] =0.22*cos(dt*count)+0.1*sin(0.5*dt*count);
                uav_command.acceleration_ref[0] = 0;
                uav_command.acceleration_ref[1] = 0;
                uav_command.acceleration_ref[2] = -0.22*sin(dt*count)+0.05*cos(0.5*dt*count);
                uav_command.yaw_ref = 0;//*/

                // // 追踪[0,0,2]
                // uav_command.Move_mode = prometheus_msgs::UAVCommand::XYZ_POS;
                // uav_command.position_ref[0] = 0;
                // uav_command.position_ref[1] = 0;
                // uav_command.position_ref[2] = target_pos[2];
                // uav_command.yaw_ref = 0;
                // 原函数，只适用于PX4_ORIGIN控制器
                // uav_command.Move_mode = prometheus_msgs::UAVCommand::XY_VEL_Z_POS; 
                // uav_command.velocity_ref[0] = -velocity*sin(count*angle_increment);
                // uav_command.velocity_ref[1] = velocity*cos(count*angle_increment);
                // uav_command.velocity_ref[2] = 0;
                // uav_command.position_ref[2] = target_pos[2];
                //发布的命令ID加1
                uav_command.Command_ID += 1;
                //发布降落命令
                uav_command_pub.publish(uav_command);
                //计数器
                if(count == control_rate*circular_time)
                {
                    circular_success = true;
                }
                count++;
                cout << GREEN << " Vx: " << uav_command.velocity_ref[0] << "  Vy: " << uav_command.velocity_ref[1] << TAIL << endl;
                // cout << GREEN << " Vx: " << uav_command.velocity_ref[0] << TAIL << endl;
                // cout << GREEN << " Vy: " << uav_command.velocity_ref[1] << TAIL << endl;

                // 无人机沿指定轨迹运动时，将command话题和state话题写入bag文件
                bag.write("/uav1/prometheus/command", ros::Time::now(), uav_command);
                bag.write("/uav1/prometheus/state", ros::Time::now(), uav_state);
                
                /* 任务完成，保持悬停，等待下一个指令 */
                if(circular_success)
                {
                    cout << GREEN << " UAV circular trajectory completed. Waiting for new command. " << TAIL << endl;
                    //时间戳
                    uav_command.header.stamp = ros::Time::now();
                    //坐标系
                    uav_command.header.frame_id = "ENU";
                    //悬停
                    uav_command.Agent_CMD = prometheus_msgs::UAVCommand::Current_Pos_Hover;
                    //发布的命令ID加1
                    uav_command.Command_ID += 1;
                    //发布命令
                    uav_command_pub.publish(uav_command);
                    // 关闭bag文件
                    bag.close();
                    // sleep(10); // 等10秒再执行下面的程序
                    // 切到此模式下等待
                    flag = 1;
                    // if(uav_control_state.control_state ==  prometheus_msgs::UAVControlState::RC_POS_CONTROL)
                    //     {   
                    //         cout << GREEN << " [circular trajectory control] tutorial_demo completed" << TAIL << endl;
                    //         cout << GREEN << "RC_POS_CONTROL and waits for next task" << TAIL << endl;
                    //     }
                    // sleep(5);
                    // //任务结束,关闭该节点
                    // ros::shutdown();
                }
                rate.sleep();
            break;
            }
            // 任务执行完毕之后的等待模式
            case 1:
            {
                rate.sleep();
                break;
            }
        
            default:
            {    rate.sleep();}
        }
        
    }
    return 0;
}