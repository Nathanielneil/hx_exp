#include <ros/ros.h>
#include <hx_msgs/UAVCommand.h>
#include <hx_msgs/UAVState.h>
#include <hx_msgs/UAVControlState.h>
#include <hx_msgs/ControllerOutput.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <iomanip>

struct ExperimentData
{
    double timestamp;
    double target_height;
    double actual_height;
    double target_x, target_y;
    double actual_x, actual_y;
    double velocity_x, velocity_y, velocity_z;
    double roll_cmd, pitch_cmd, yaw_cmd, throttle_cmd;
    int controller_type;
    std::string controller_name;
    
    // Calculated metrics
    double horizontal_drift;
    double height_error;
    double velocity_error;
};

class GroundEffectExperiment
{
public:
    GroundEffectExperiment(ros::NodeHandle& nh) : nh_(nh), experiment_phase_(0), 
                                                 phase_start_time_(0), current_controller_(1)
    {
        // Parameters
        nh_.param<double>("experiment/takeoff_height", takeoff_height_, 3.0);
        nh_.param<double>("experiment/landing_speed", landing_speed_, 0.5);
        nh_.param<double>("experiment/hover_duration", hover_duration_, 5.0);
        nh_.param<std::string>("experiment/data_file", data_filename_, "ground_effect_data.csv");
        nh_.param<bool>("experiment/auto_repeat", auto_repeat_, true);
        
        // Publishers
        command_pub_ = nh_.advertise<hx_msgs::UAVCommand>("/Drone1/hx_uav/command", 10);
        
        // Subscribers
        state_sub_ = nh_.subscribe<hx_msgs::UAVState>("/Drone1/hx_uav/state", 10,
                                                     &GroundEffectExperiment::stateCallback, this);
        control_state_sub_ = nh_.subscribe<hx_msgs::UAVControlState>("/Drone1/hx_uav/control_state", 10,
                                                                   &GroundEffectExperiment::controlStateCallback, this);
        controller_output_sub_ = nh_.subscribe<hx_msgs::ControllerOutput>("/Drone1/hx_uav/controller_output", 10,
                                                                        &GroundEffectExperiment::controllerOutputCallback, this);
        
        // Timer for experiment execution
        experiment_timer_ = nh_.createTimer(ros::Duration(0.05), // 20Hz
                                          &GroundEffectExperiment::experimentTimerCallback, this);
        
        // Initialize data file
        initializeDataFile();
        
        // Store takeoff position
        takeoff_position_recorded_ = false;
        
        ROS_INFO("[Ground Effect Experiment] Initialized");
        ROS_INFO("[Ground Effect Experiment] Takeoff Height: %.1f m", takeoff_height_);
        ROS_INFO("[Ground Effect Experiment] Landing Speed: %.2f m/s", landing_speed_);
        ROS_INFO("[Ground Effect Experiment] Data File: %s", data_filename_.c_str());
    }
    
    ~GroundEffectExperiment()
    {
        if (data_file_.is_open())
        {
            data_file_.close();
        }
        
        // Generate analysis report
        generateAnalysisReport();
    }
    
private:
    ros::NodeHandle nh_;
    ros::Publisher command_pub_;
    ros::Subscriber state_sub_;
    ros::Subscriber control_state_sub_;
    ros::Subscriber controller_output_sub_;
    ros::Timer experiment_timer_;
    
    // Experiment parameters
    double takeoff_height_;
    double landing_speed_;
    double hover_duration_;
    std::string data_filename_;
    bool auto_repeat_;
    
    // Experiment state
    int experiment_phase_;
    ros::Time phase_start_time_;
    int current_controller_;
    bool takeoff_position_recorded_;
    geometry_msgs::Point takeoff_position_;
    
    // UAV state
    hx_msgs::UAVState current_state_;
    hx_msgs::UAVControlState current_control_state_;
    hx_msgs::ControllerOutput current_controller_output_;
    bool state_received_;
    bool control_state_received_;
    bool controller_output_received_;
    
    // Data recording
    std::ofstream data_file_;
    std::vector<ExperimentData> experiment_data_;
    
    void stateCallback(const hx_msgs::UAVState::ConstPtr& msg)
    {
        current_state_ = *msg;
        state_received_ = true;
        
        // Record takeoff position
        if (!takeoff_position_recorded_ && msg->connected)
        {
            takeoff_position_.x = msg->position[0];
            takeoff_position_.y = msg->position[1];
            takeoff_position_.z = msg->position[2];
            takeoff_position_recorded_ = true;
            ROS_INFO("[Experiment] Takeoff position recorded: [%.2f, %.2f, %.2f]",
                     takeoff_position_.x, takeoff_position_.y, takeoff_position_.z);
        }
    }
    
    void controlStateCallback(const hx_msgs::UAVControlState::ConstPtr& msg)
    {
        current_control_state_ = *msg;
        control_state_received_ = true;
    }
    
    void controllerOutputCallback(const hx_msgs::ControllerOutput::ConstPtr& msg)
    {
        current_controller_output_ = *msg;
        controller_output_received_ = true;
    }
    
    void experimentTimerCallback(const ros::TimerEvent& e)
    {
        if (!state_received_ || !control_state_received_)
        {
            return;
        }
        
        ros::Time current_time = ros::Time::now();
        
        // Record data during landing phase
        if (experiment_phase_ == 2)
        {
            recordExperimentData(current_time);
        }
        
        switch (experiment_phase_)
        {
            case 0: // Takeoff phase
                handleTakeoffPhase(current_time);
                break;
                
            case 1: // Hover phase
                handleHoverPhase(current_time);
                break;
                
            case 2: // Landing phase
                handleLandingPhase(current_time);
                break;
                
            case 3: // Analysis and switch controller
                handleAnalysisPhase(current_time);
                break;
                
            case 4: // Experiment complete
                handleCompletionPhase();
                break;
                
            default:
                break;
        }
    }
    
    void handleTakeoffPhase(const ros::Time& current_time)
    {
        if (phase_start_time_.isZero())
        {
            ROS_INFO("[Experiment] Phase 1: Takeoff with %s controller", 
                     getControllerName(current_controller_).c_str());
            sendTakeoffCommand(current_controller_);
            phase_start_time_ = current_time;
        }
        else if ((current_time - phase_start_time_).toSec() > 5.0 && 
                 current_control_state_.control_state == hx_msgs::UAVControlState::COMMAND_CONTROL &&
                 current_state_.position[2] >= takeoff_height_ - 0.5)
        {
            experiment_phase_ = 1;
            phase_start_time_ = ros::Time(0);
        }
    }
    
    void handleHoverPhase(const ros::Time& current_time)
    {
        if (phase_start_time_.isZero())
        {
            ROS_INFO("[Experiment] Phase 2: Hover stabilization");
            sendHoverCommand(current_controller_);
            phase_start_time_ = current_time;
        }
        else if ((current_time - phase_start_time_).toSec() > hover_duration_)
        {
            experiment_phase_ = 2;
            phase_start_time_ = ros::Time(0);
        }
    }
    
    void handleLandingPhase(const ros::Time& current_time)
    {
        if (phase_start_time_.isZero())
        {
            ROS_INFO("[Experiment] Phase 3: Landing test with %s controller (Speed: %.2f m/s)", 
                     getControllerName(current_controller_).c_str(), landing_speed_);
            phase_start_time_ = current_time;
        }
        
        // Calculate target height based on landing speed
        double elapsed_time = (current_time - phase_start_time_).toSec();
        double target_height = takeoff_height_ - (landing_speed_ * elapsed_time);
        
        if (target_height <= 0.2)
        {
            // Landing complete
            sendLandCommand();
            experiment_phase_ = 3;
            phase_start_time_ = ros::Time(0);
        }
        else
        {
            // Continue controlled descent
            sendMoveCommand(takeoff_position_.x, takeoff_position_.y, target_height, current_controller_);
        }
    }
    
    void handleAnalysisPhase(const ros::Time& current_time)
    {
        if (phase_start_time_.isZero())
        {
            ROS_INFO("[Experiment] Phase 4: Landing complete, analyzing data...");
            analyzeCurrentExperiment();
            phase_start_time_ = current_time;
        }
        else if ((current_time - phase_start_time_).toSec() > 3.0)
        {
            // Switch to next controller
            current_controller_++;
            
            if (current_controller_ <= 3 && auto_repeat_)
            {
                ROS_INFO("[Experiment] Switching to next controller (%d/3)", current_controller_);
                experiment_phase_ = 0;  // Restart with next controller
                phase_start_time_ = ros::Time(0);
            }
            else
            {
                experiment_phase_ = 4;  // All controllers tested
            }
        }
    }
    
    void handleCompletionPhase()
    {
        ROS_INFO("[Experiment] All controllers tested! Check results in: %s", data_filename_.c_str());
        generateFinalReport();
        ros::shutdown();
    }
    
    void sendTakeoffCommand(int controller_type)
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Takeoff;
        cmd.Control_mode = controller_type;
        cmd.Command_ID = "ground_effect_takeoff";
        
        command_pub_.publish(cmd);
    }
    
    void sendHoverCommand(int controller_type)
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Hold;
        cmd.Control_mode = controller_type;
        cmd.Command_ID = "ground_effect_hover";
        
        command_pub_.publish(cmd);
    }
    
    void sendMoveCommand(double x, double y, double z, int controller_type)
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Move_ENU;
        cmd.Control_mode = controller_type;
        
        cmd.position.x = x;
        cmd.position.y = y;
        cmd.position.z = z;
        
        // Set downward velocity for controlled descent
        cmd.linear_vel.x = 0.0;
        cmd.linear_vel.y = 0.0;
        cmd.linear_vel.z = -landing_speed_;
        
        cmd.Command_ID = "ground_effect_landing";
        
        command_pub_.publish(cmd);
    }
    
    void sendLandCommand()
    {
        hx_msgs::UAVCommand cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.Agent_CMD = hx_msgs::UAVCommand::Land;
        cmd.Command_ID = "ground_effect_land";
        
        command_pub_.publish(cmd);
    }
    
    void recordExperimentData(const ros::Time& current_time)
    {
        if (!controller_output_received_)
            return;
            
        ExperimentData data;
        
        // Time stamp
        data.timestamp = current_time.toSec();
        
        // Heights
        data.actual_height = current_state_.position[2];
        double elapsed_time = (current_time - phase_start_time_).toSec();
        data.target_height = takeoff_height_ - (landing_speed_ * elapsed_time);
        
        // Positions
        data.actual_x = current_state_.position[0];
        data.actual_y = current_state_.position[1];
        data.target_x = takeoff_position_.x;
        data.target_y = takeoff_position_.y;
        
        // Velocities
        data.velocity_x = current_state_.velocity[0];
        data.velocity_y = current_state_.velocity[1];
        data.velocity_z = current_state_.velocity[2];
        
        // Control commands
        data.roll_cmd = current_controller_output_.roll_command;
        data.pitch_cmd = current_controller_output_.pitch_command;
        data.yaw_cmd = current_controller_output_.yaw_rate_command;
        data.throttle_cmd = current_controller_output_.throttle_command;
        
        // Controller info
        data.controller_type = current_controller_;
        data.controller_name = getControllerName(current_controller_);
        
        // Calculate metrics
        data.horizontal_drift = sqrt(pow(data.actual_x - data.target_x, 2) + 
                                   pow(data.actual_y - data.target_y, 2));
        data.height_error = abs(data.actual_height - data.target_height);
        data.velocity_error = abs(data.velocity_z - (-landing_speed_));
        
        // Store data
        experiment_data_.push_back(data);
        
        // Write to file
        if (data_file_.is_open())
        {
            data_file_ << std::fixed << std::setprecision(3)
                      << data.timestamp << ","
                      << data.controller_name << ","
                      << data.target_height << ","
                      << data.actual_height << ","
                      << data.target_x << ","
                      << data.actual_x << ","
                      << data.target_y << ","
                      << data.actual_y << ","
                      << data.velocity_x << ","
                      << data.velocity_y << ","
                      << data.velocity_z << ","
                      << data.roll_cmd << ","
                      << data.pitch_cmd << ","
                      << data.throttle_cmd << ","
                      << data.horizontal_drift << ","
                      << data.height_error << ","
                      << data.velocity_error << std::endl;
        }
    }
    
    void initializeDataFile()
    {
        data_file_.open(data_filename_);
        if (data_file_.is_open())
        {
            // Write CSV header
            data_file_ << "timestamp,controller,target_height,actual_height,"
                      << "target_x,actual_x,target_y,actual_y,"
                      << "velocity_x,velocity_y,velocity_z,"
                      << "roll_cmd,pitch_cmd,throttle_cmd,"
                      << "horizontal_drift,height_error,velocity_error" << std::endl;
        }
        else
        {
            ROS_ERROR("[Experiment] Failed to open data file: %s", data_filename_.c_str());
        }
    }
    
    void analyzeCurrentExperiment()
    {
        if (experiment_data_.empty())
            return;
            
        // Filter data for current controller
        std::vector<ExperimentData> current_data;
        for (const auto& data : experiment_data_)
        {
            if (data.controller_type == current_controller_)
            {
                current_data.push_back(data);
            }
        }
        
        if (current_data.empty())
            return;
            
        // Calculate statistics
        double avg_horizontal_drift = 0, max_horizontal_drift = 0;
        double avg_height_error = 0, max_height_error = 0;
        double avg_velocity_error = 0, max_velocity_error = 0;
        double avg_roll_variation = 0, avg_pitch_variation = 0;
        
        // Calculate averages
        for (const auto& data : current_data)
        {
            avg_horizontal_drift += data.horizontal_drift;
            avg_height_error += data.height_error;
            avg_velocity_error += data.velocity_error;
            
            max_horizontal_drift = std::max(max_horizontal_drift, data.horizontal_drift);
            max_height_error = std::max(max_height_error, data.height_error);
            max_velocity_error = std::max(max_velocity_error, data.velocity_error);
        }
        
        int n = current_data.size();
        avg_horizontal_drift /= n;
        avg_height_error /= n;
        avg_velocity_error /= n;
        
        // Calculate control command variations
        if (n > 1)
        {
            double roll_sum = 0, pitch_sum = 0;
            for (int i = 1; i < n; i++)
            {
                roll_sum += abs(current_data[i].roll_cmd - current_data[i-1].roll_cmd);
                pitch_sum += abs(current_data[i].pitch_cmd - current_data[i-1].pitch_cmd);
            }
            avg_roll_variation = roll_sum / (n - 1);
            avg_pitch_variation = pitch_sum / (n - 1);
        }
        
        // Final landing accuracy
        double final_landing_error = current_data.back().horizontal_drift;
        
        // Print results
        ROS_INFO("=== %s Controller Ground Effect Analysis ===", 
                 getControllerName(current_controller_).c_str());
        ROS_INFO("Average Horizontal Drift: %.3f m", avg_horizontal_drift);
        ROS_INFO("Maximum Horizontal Drift: %.3f m", max_horizontal_drift);
        ROS_INFO("Average Height Error: %.3f m", avg_height_error);
        ROS_INFO("Average Velocity Error: %.3f m/s", avg_velocity_error);
        ROS_INFO("Average Roll Variation: %.3f rad", avg_roll_variation);
        ROS_INFO("Average Pitch Variation: %.3f rad", avg_pitch_variation);
        ROS_INFO("Final Landing Accuracy: %.3f m", final_landing_error);
        ROS_INFO("Data Points Collected: %d", n);
    }
    
    void generateFinalReport()
    {
        std::string report_filename = "ground_effect_report.txt";
        std::ofstream report_file(report_filename);
        
        if (report_file.is_open())
        {
            report_file << "Ground Effect Experiment Final Report\n";
            report_file << "=====================================\n\n";
            
            report_file << "Experiment Parameters:\n";
            report_file << "- Takeoff Height: " << takeoff_height_ << " m\n";
            report_file << "- Landing Speed: " << landing_speed_ << " m/s\n";
            report_file << "- Hover Duration: " << hover_duration_ << " s\n\n";
            
            // Analyze each controller
            for (int controller = 1; controller <= 3; controller++)
            {
                analyzeControllerPerformance(report_file, controller);
            }
            
            report_file.close();
            ROS_INFO("[Experiment] Final report saved to: %s", report_filename.c_str());
        }
    }
    
    void analyzeControllerPerformance(std::ofstream& report_file, int controller_type)
    {
        // Filter data for specific controller
        std::vector<ExperimentData> controller_data;
        for (const auto& data : experiment_data_)
        {
            if (data.controller_type == controller_type)
            {
                controller_data.push_back(data);
            }
        }
        
        if (controller_data.empty())
            return;
            
        report_file << getControllerName(controller_type) << " Controller Analysis:\n";
        report_file << "----------------------------------------\n";
        
        // Calculate comprehensive statistics
        // ... (implementation details)
        
        report_file << "\n";
    }
    
    void generateAnalysisReport()
    {
        ROS_INFO("[Experiment] Generating comprehensive analysis report...");
        
        // This could include:
        // - Statistical analysis
        // - Performance ranking
        // - Recommendations
        // - Graph generation (if matplotlib-cpp available)
    }
    
    std::string getControllerName(int controller_type)
    {
        switch (controller_type)
        {
            case 1: return "PID";
            case 2: return "UDE"; 
            case 3: return "ADRC";
            default: return "Unknown";
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ground_effect_experiment");
    ros::NodeHandle nh("~");
    
    ROS_INFO("[Ground Effect Experiment] Starting ground effect comparison experiment...");
    
    GroundEffectExperiment experiment(nh);
    
    ros::spin();
    
    return 0;
}