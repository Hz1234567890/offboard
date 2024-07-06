#include "OffboardControl.hpp"

OffboardControl::OffboardControl()
:Node("OffboardControl")
{
    twist_stamped_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 5);
    state_sub = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10, 
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                state_callback(msg);
            });
    local_pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 5);
    // timer_ = this->create_wall_timer(
    //     std::chrono::seconds(1),
    //     std::bind(&OffboardControl::timer_callback, this));
}

void OffboardControl::run(){
    RCLCPP_INFO(this->get_logger(), "现在开始run函数");
    init();
    RCLCPP_INFO(this->get_logger(), "结束init函数");
    takeoff(3);//起飞高度3米    
    send_local_setpoint_command(5,10,2,90);
    send_velocity_command(2,0,0);
}

void OffboardControl::state_callback(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state = *msg;
}

// void OffboardControl::timer_callback(){
//     RCLCPP_INFO(this->get_logger(), "现在开始timer_callback函数");
//     init();
//     RCLCPP_INFO(this->get_logger(), "结束init函数");
// }
