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
#include <mavros_msgs/srv/command_home.hpp>
#include <mavros_msgs/msg/home_position.hpp>
#include <cmath>

#define PI 3.14159265358979323846


class OffboardControl : public rclcpp::Node{
public:
    OffboardControl();
    //初始化运动节点
    void init();
    //起飞函数s
    void takeoff(int halt);
    //控制线速度
    void send_velocity_command(double linear_x, double linear_y, double linear_z);
    //控制线速度with time
    void send_velocity_command_with_time(double linear_x, double linear_y, double linear_z,int time);
    //控制位置
    void send_local_setpoint_command(double x, double y, double z,double angle);
    //坐标转换函数
    void dxyToGlobal(double dx, double dy, double headingangle_compass, double& x, double& y, double& angle);
    mavros_msgs::msg::State current_state;
    //定时器回调函数
    void run();
    // void timer_callback(void);
    
    //设置家地址
    void set_home_position();

    //环绕投弹区
    void surround_shot(double x,double y,double length,double width);
    //环绕侦查区
    void surround_see(double x,double y,double length,double width);

    
private:
    typedef struct{
		double x;
		double y;
		double z;
	}Location;
	Location home_position;
    const double headingangle_compass = 180.0;
    const double shot_length = 7.0;
    const double shot_width = 5.0;
    const double shot_halt = 2.5;
    const double see_length = 7.0;
    const double see_width = 5.0;
    const double see_halt = 4.0;
    // auto node = rclcpp::Node::make_shared("offboard");
    // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher;
    rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_position_subscription_;
	
    
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
    void home_position_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg);
    
    // geometry_msgs::msg::PoseStamped last_pose_;
    // rclcpp::Time last_update_time_;
    // 无人机状态回调函数
    
    //声明定时器
    rclcpp::TimerBase::SharedPtr timer_;

    

};