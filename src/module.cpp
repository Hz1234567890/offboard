#include "OffboardControl.hpp"

void OffboardControl::Doshot()
{
    RCLCPP_INFO(this->get_logger(), "Doshot");
    auto Doshot_start = std::chrono::system_clock::now();
    auto start = std::chrono::system_clock::now();
    int surround_shot_count = 0;
    while (this->yolo->get_flag() == 0 || this->yolo->get_halt() > 1.5)
    {
        // RCLCPP_INFO(this->get_logger(), "当前高度：%lf",this->yolo->get_halt());
        auto now = std::chrono::system_clock::now();
        if (now - Doshot_start > std::chrono::seconds(60))
        {
            RCLCPP_INFO(this->get_logger(), "超时");
            break;
        }
        if (this->yolo->get_x() == 0 && this->yolo->get_y() == 0)
        {

            if (now - start > std::chrono::seconds(1))
            { // send_velocity_command(0, 0, 0);
                RCLCPP_INFO(this->get_logger(), "前往下一点");
                surround_shot_goto_next(dx_shot, dy_shot, shot_length, shot_width);
                start = std::chrono::system_clock::now();
                surround_shot_count++;
                if (surround_shot_count > 12)
                {
                    break;
                }
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "看见桶了，执行PID");
            PID(this->yolo->get_x(), this->yolo->get_y(), this->yolo->get_halt(), target_x, target_y, target_z, accuracy, z_accuracy, k, kp, ki, kd, dt);
        }
    }
    // send_velocity_command_with_time(0, 0, 0, 1);
    RCLCPP_INFO(this->get_logger(), "投弹!!投弹!!");
    servo_controller(12, 1800);
    rclcpp::sleep_for(std::chrono::seconds(1));
}

void OffboardControl::surround_shot_goto_next(double x, double y, double length, double width)
{
    static int surround_shot_cnt = 0;
    if (surround_shot_cnt > 12)
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
    double per_time = 10;
    send_velocity_command_with_time(length / 2.0 / per_time, width / per_time, 0, per_time);
    RCLCPP_INFO(this->get_logger(), "侦查区点位1_1 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);

    send_velocity_command_with_time(0, -width / per_time, 0, per_time);
    RCLCPP_INFO(this->get_logger(), "侦查区点位2_1 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);

    send_velocity_command_with_time(-length / 2.0 / per_time, width / per_time, 0, per_time);
    RCLCPP_INFO(this->get_logger(), "侦查区点位3_1 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);

    send_velocity_command_with_time(-length / 2.0 / per_time, -width / per_time, 0, per_time);
    RCLCPP_INFO(this->get_logger(), "侦查区点位4_1 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);

    send_velocity_command_with_time(0, width / per_time, 0, per_time);
    RCLCPP_INFO(this->get_logger(), "侦查区点位5_1 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);

    send_velocity_command_with_time(length / 2.0 / per_time, -width / per_time, 0, per_time);
    RCLCPP_INFO(this->get_logger(), "侦查区点位6_1 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
}

void OffboardControl::Doland()
{
    RCLCPP_INFO(this->get_logger(), "Doland");
    double x_home = 0.0, y_home = 0.0, angle = headingangle_compass;
    dxyToGlobal(0, 2, headingangle_compass, x_home, y_home, angle);
    RCLCPP_INFO(this->get_logger(), "返回降落准备点 x: %lf   y: %lf    angle: %lf", x_home, y_home, angle);
    send_local_setpoint_command(x_home, y_home, 2, angle);

    dxyToGlobal(0, 0.5, headingangle_compass, x_home, y_home, angle);
    RCLCPP_INFO(this->get_logger(), "返回降落点 x: %lf   y: %lf    angle: %lf", x_home, y_home, angle);
    auto Doland_start = std::chrono::system_clock::now();
    auto start = std::chrono::system_clock::now();
    bool is_land = false;
    static int surround_land = -1;
    while (!is_land)
    {
        auto now = std::chrono::system_clock::now();
        if (now - Doland_start > std::chrono::seconds(20) || surround_land > 1)
        {
            break;
        }
        if (this->yolo->get_x() == 0 && this->yolo->get_y() == 0)
        {
            if (now - start > std::chrono::seconds(1))
            {
                RCLCPP_INFO(this->get_logger(), "surround_land = %d", surround_land);
                send_local_setpoint_command(x_home + surround_land, y_home, 3, angle);
                start = std::chrono::system_clock::now();
                surround_land++;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "看见H了，执行PID_rtl");
            PID_rtl(this->yolo->get_x(), this->yolo->get_y(), now_halt, target_x, target_y, is_land);
        }
    }
    send_velocity_command_with_time(0, 0, 0, 1);
    RCLCPP_INFO(this->get_logger(), "1s后降落");
    set_mode("LAND");
}