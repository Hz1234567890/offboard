#include <iostream>  
#include <string>  

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include "mavros_msgs/msg/global_position_target.hpp"

class OffboardControl : public rclcpp::Node{
public:
    OffboardControl(const std::string ardupilot_namespace,std::shared_ptr<YOLO> yolo_,std::shared_ptr<ServoController> servo_controller_) :
        Node("offboard"),
        twist_stamped_publisher_{this->create_publisher<geometry_msgs::msg::TwistStamped>(ardupilot_namespace+"setpoint_velocity/cmd_vel", 5)},    
        local_setpoint_publisher_{this->create_publisher<geometry_msgs::msg::PoseStamped>(ardupilot_namespace+"setpoint_position/local", 5)},
        local_pos_pub_{this->create_publisher<geometry_msgs::msg::PoseStamped>(ardupilot_namespace+"setpoint_position/local", 5)}

    {
        //初始化运动节点
        void init(int argc, char **argv);
        //起飞函数
        void takeoff(int halt);
        //控制线速度
        void send_velocity_command(double linear_x, double linear_y, double linear_z, double angular_z);
        //控制线速度with time
        void send_velocity_command_with_time(double linear_x, double linear_y, double linear_z, double angular_z,double time);
        //控制位置
        void send_local_setpoint_command(double x, double y, double z,double angle);
        //定时器回调函数
        void timer_callback(void);
    }   
private:
    {
        // auto node = rclcpp::Node::make_shared("offboard");
        // 无人机状态回调函数
        void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
            current_state = *msg;
        }
        auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
            "/mavros/state", 10, state_callback);

        
    }

};

