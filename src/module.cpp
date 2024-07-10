#include "OffboardControl.hpp"

void OffboardControl::Doshot()
{
    RCLCPP_INFO(this->get_logger(), "Doshot");
    auto Doshot_start = std::chrono::system_clock::now();
    auto start = std::chrono::system_clock::now();
    while (this->yolo->get_flag() == 0)
    {
        auto now = std::chrono::system_clock::now();
        if(now-Doshot_start > std::chrono::seconds(40)){
            break;
        }
        if (this->yolo->get_x() == 0 && this->yolo->get_y() == 0)
        {
            if (now - start > std::chrono::seconds(3)){
                surround_shot_goto_next(dx_shot,dy_shot,shot_length,shot_width);
                start = std::chrono::system_clock::now();
            }
        }
        else
        {
            PID(this->yolo->get_x(), this->yolo->get_y(), now_halt, target_x, target_y, target_z, accuracy, z_accuracy, k,kp, ki, kd, dt);
        }
    }
    send_velocity_command_with_time(0,0,0,2);
    RCLCPP_INFO(this->get_logger(), "2s后投弹");
    servo_controller(12,1800);
    rclcpp::sleep_for(std::chrono::seconds(1));
}

void OffboardControl::surround_shot_goto_next(double x, double y, double length, double width)
{
    static int surround_shot_cnt = 0;
    if (surround_shot_cnt > 7)
    {
        RCLCPP_INFO(this->get_logger(), "投弹区与已经全部遍历");
        return;
    }
    double x_shot = 0.0, y_shot = 0.0, angle = headingangle_compass;
    dxyToGlobal(x + (length * surround_shot_points[surround_shot_cnt].dx), y + (width * surround_shot_points[surround_shot_cnt].dy), headingangle_compass, x_shot, y_shot, angle);
    RCLCPP_INFO(this->get_logger(), "投弹区点位1 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
    send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(3));

    surround_shot_cnt++;
}

// void OffboardControl::surround_shot(double x,double y,double length,double width){
//     double x_shot = 0.0, y_shot = 0.0,angle=headingangle_compass;
//     dxyToGlobal(x+(length/2.0), y, headingangle_compass, x_shot, y_shot, angle);
//     RCLCPP_INFO(this->get_logger(), "投弹区点位1 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
//     send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);
//     rclcpp::sleep_for(std::chrono::seconds(3));
//     dxyToGlobal(x+(length/2.0), y+width, headingangle_compass, x_shot, y_shot, angle);
//     RCLCPP_INFO(this->get_logger(), "投弹区点位2 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
//     send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);
//     rclcpp::sleep_for(std::chrono::seconds(3));
//     dxyToGlobal(x-(length/2.0), y+width, headingangle_compass, x_shot, y_shot, angle);
//     RCLCPP_INFO(this->get_logger(), "投弹区点位3 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
//     send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);
//     rclcpp::sleep_for(std::chrono::seconds(3));
//     dxyToGlobal(x-(length/2.0), y, headingangle_compass, x_shot, y_shot, angle);
//     RCLCPP_INFO(this->get_logger(), "投弹区点位4 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
//     send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);

//     rclcpp::sleep_for(std::chrono::seconds(3));
//     dxyToGlobal(x, y, headingangle_compass, x_shot, y_shot, angle);
//     RCLCPP_INFO(this->get_logger(), "投弹区点位5 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
//     send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);
// }

void OffboardControl::surround_see(double x, double y, double length, double width)
{
    double x_see = 0.0, y_see = 0.0, angle = headingangle_compass;
    dxyToGlobal(x + (length / 2.0), y, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位1 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(3));
    dxyToGlobal(x + (length / 2.0), y + width, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位2 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(3));
    dxyToGlobal(x - (length / 2.0), y + width, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位3 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(3));
    dxyToGlobal(x - (length / 2.0), y, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位4 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::seconds(3));
    dxyToGlobal(x, y, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位5 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
}