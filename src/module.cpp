#include "OffboardControl.hpp"
void OffboardControl::surround_shot(double x,double y,double length,double width){
    double x_shot = 0.0, y_shot = 0.0,angle=headingangle_compass;
    dxyToGlobal(x+(length/2.0), y, headingangle_compass, x_shot, y_shot, angle);
    RCLCPP_INFO(this->get_logger(), "投弹区点位1 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
    send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(3));
    dxyToGlobal(x+(length/2.0), y+width, headingangle_compass, x_shot, y_shot, angle);
    RCLCPP_INFO(this->get_logger(), "投弹区点位2 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
    send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(3));
    dxyToGlobal(x-(length/2.0), y+width, headingangle_compass, x_shot, y_shot, angle);
    RCLCPP_INFO(this->get_logger(), "投弹区点位3 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
    send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(3));
    dxyToGlobal(x-(length/2.0), y, headingangle_compass, x_shot, y_shot, angle);
    RCLCPP_INFO(this->get_logger(), "投弹区点位4 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
    send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);

    rclcpp::sleep_for(std::chrono::seconds(3));
    dxyToGlobal(x, y, headingangle_compass, x_shot, y_shot, angle);
    RCLCPP_INFO(this->get_logger(), "投弹区点位5 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
    send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);
}

void OffboardControl::surround_see(double x,double y,double length,double width){
    double x_see = 0.0, y_see = 0.0,angle=headingangle_compass;
    dxyToGlobal(x+(length/2.0), y, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位1 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(3));
    dxyToGlobal(x+(length/2.0), y+width, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位2 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(3));
    dxyToGlobal(x-(length/2.0), y+width, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位3 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(3));
    dxyToGlobal(x-(length/2.0), y, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位4 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(3));
    dxyToGlobal(x, y, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位5 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);

}