#include "OffboardControl.hpp"

OffboardControl::OffboardControl()
    : Node("OffboardControl")
{
    twist_stamped_publisher = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 5);
    state_sub = this->create_subscription<mavros_msgs::msg::State>("/mavros/state", 10,
                                                                   [this](const mavros_msgs::msg::State::SharedPtr msg)
                                                                   {
                                                                       state_callback(msg);
                                                                   });
    local_pos_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 5);
    // home_position_subscription_ = this->create_subscription<mavros_msgs::msg::HomePosition>("mavros/home_position/home", 10,
    // 		std::bind(&OffboardControl::home_position_callback, this, std::placeholders::_1));
    // timer_ = this->create_wall_timer(
    //     std::chrono::seconds(1),
    //     std::bind(&OffboardControl::timer_callback, this));
}

void OffboardControl::run()
{
    RCLCPP_INFO(this->get_logger(), "现在开始run函数");
    init();
    RCLCPP_INFO(this->get_logger(), "结束init函数");
    takeoff(3); // 起飞高度3米
    double dx_shot = 0.0, dy_shot = 30.0;
    double dx_see = 0.0, dy_see = 55.0;
    double x_shot = 0.0, y_shot = 0.0, angle = headingangle_compass;
    double x_see = 0.0, y_see = 0.0;//angle角度同shot。
    // headingangle_compass为罗盘读数
    // angle为四元数的角度，
    // dx,dy为飞机坐标系下的横纵坐标
    // x,y为global坐标下的，x正指向正东，y正指向正北
    dxyToGlobal(dx_shot, dy_shot, headingangle_compass, x_shot, y_shot, angle);
    RCLCPP_INFO(this->get_logger(), "投弹区起点 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
    dxyToGlobal(dx_see,dy_see,headingangle_compass,x_see,y_see,angle);
    RCLCPP_INFO(this->get_logger(), "侦查起点 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "开始前往投弹区起点");
    send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(5));
    surround_shot(dx_shot,dy_shot,shot_length,shot_width);
    rclcpp::sleep_for(std::chrono::seconds(5));
    /**
     * 这里需要加入PID的函数
    */

    RCLCPP_INFO(this->get_logger(), "开始前往侦查起点");   
    send_local_setpoint_command(x_see,y_see,see_halt,angle);
    rclcpp::sleep_for(std::chrono::seconds(5));
    surround_see(dx_see,dy_see,see_length,see_width);
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

void OffboardControl::home_position_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg)
{
    home_position.x = msg->position.x;
    home_position.y = msg->position.y;
    home_position.z = msg->position.z;
    // RCLCPP_INFO(this->get_logger(), "Received home position data");
    // RCLCPP_INFO(this->get_logger(), "Latitude: %f", msg->geo.latitude);
    // RCLCPP_INFO(this->get_logger(), "Longitude: %f", msg->geo.longitude);
    // RCLCPP_INFO(this->get_logger(), "Altitude: %f", msg->geo.altitude);
    // RCLCPP_INFO(this->get_logger(), "X: %f", msg->position.x);
    // RCLCPP_INFO(this->get_logger(), "Y: %f", msg->position.y);
    // RCLCPP_INFO(this->get_logger(), "Z: %f", msg->position.z);
}

void OffboardControl::dxyToGlobal(double dx, double dy, double headingangle_compass, double& x, double& y, double& angle) {
    angle = 450.0 - headingangle_compass;
    if (angle > 360.0) {
        angle = angle - 360.0;
    }
    double radians_angle = angle * PI / 180.0;
    double radians_angle_90 = (angle - 90) * PI / 180.0;
    x = dy * cos(radians_angle) + dx * cos(radians_angle_90);
    y = dy * sin(radians_angle) + dx * sin(radians_angle_90);
}