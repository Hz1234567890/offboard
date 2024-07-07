#include <iostream>  
#include <string> 
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include "mavros_msgs/msg/global_position_target.hpp"
#include <cmath>
#define PI 3.14159


class OffboardControl : public rclcpp::Node{
public:
    OffboardControl();
    //初始化运动节点
    void init();
    //起飞函数
    void takeoff(int halt);
    //控制线速度
    void send_velocity_command(double linear_x, double linear_y, double linear_z);
    //控制线速度with time
    void send_velocity_command_with_time(double linear_x, double linear_y, double linear_z,int time);
    //控制位置
    void send_local_setpoint_command(double x, double y, double z,double angle);
    //定时器回调函数
    void run();
    // void timer_callback(void);
    mavros_msgs::msg::State current_state;
    
private:
    // auto node = rclcpp::Node::make_shared("offboard");
    // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher;
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg);

    
    // geometry_msgs::msg::PoseStamped last_pose_;
    // rclcpp::Time last_update_time_;
    // 无人机状态回调函数
    
    //声明定时器
    rclcpp::TimerBase::SharedPtr timer_;

    

};