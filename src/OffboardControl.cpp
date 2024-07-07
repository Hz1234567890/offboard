#include "OffboardControl.hpp"

const double headingangle_compass=185.0;

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
    double dx = 0.0, dy = 30.0;
    double angle = 450.0 - headingangle_compass;
    if (angle>360.0){
        angle=angle-360.0;
    }  
    double radians_angle = angle * PI / 180.0;
    double radians_angle_90 = (angle-90) * PI /180.0;
    double x=dy*cos(radians_angle)+dx*cos(radians_angle_90);
    double y=dy*sin(radians_angle)+dx*sin(radians_angle_90);
    RCLCPP_INFO(node->get_logger(), "x: %lf   y: %lf", x, y);
    send_local_setpoint_command(x,y,2,angle);
    // rclcpp::sleep_for(std::chrono::seconds(10));
    // send_velocity_command(2,0,0);
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
