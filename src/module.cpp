#include "OffboardControl.hpp"

// void OffboardControl::Doshot()
// {
//     RCLCPP_INFO(this->get_logger(), "Doshot");
//     auto Doshot_start = std::chrono::system_clock::now();
//     auto start = std::chrono::system_clock::now();
//     int surround_shot_count = 0;

//     double g_v_x = 0, g_v_y = 0;

//     int shot_point_cnt = 0; //现在是第几条边，1是第一条边
//     //每条边的速度和时间
//     //                                    1     2     3    4    5     6     7    8    9    10    11   12
//     double shot_point_vel_x[20]    ={0, 0.5,    0, -0.5,   0, 0.5,    0, -0.5,   0, 0.5,    0};
//     double shot_point_vel_y[20]    ={0,   0,  0.5,    0,-0.5,   0,  0.5,    0,-0.5,   0,  0.5};
//     double shot_point_time[20]     ={0,   3,    2,    6,   4,   9,    6,   12,   8,  14,    9};
//     double shot_point_real_x = 0, shot_point_real_y = 0;
//     int pid_lost = 0; //pid丢失(PID是否执行过，1代表执行过)

//     while (this->yolo->get_flag() == 0 || this->yolo->get_halt() > 1.5)
//     {
//         // RCLCPP_INFO(this->get_logger(), "当前高度：%lf",this->yolo->get_halt());
//         auto now = std::chrono::system_clock::now();
//         if (now - Doshot_start > std::chrono::seconds(60))
//         {
//             RCLCPP_INFO(this->get_logger(), "超时");
//             break;
//         }
//         if (this->yolo->get_x() == 0 && this->yolo->get_y() == 0)
//         {
//             if (pid_lost) {
//                     pid_lost = 0;

//                     double x_shot = 0.0, y_shot = 0.0,angle=headingangle_compass;
//                     dxyToGlobal(shot_point_real_x, shot_point_real_y, headingangle_compass, x_shot, y_shot, angle);
//                     RCLCPP_INFO(this->get_logger(), "投弹区点位1 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
//                     send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);

//                     // send_local_setpoint_command(shot_point_real_x, shot_point_real_y, shot_halt);
//                     rclcpp::sleep_for(std::chrono::seconds(3));
//                     // send_velocity_command(shot_point_vel_x[shot_point_cnt], shot_point_vel_y[shot_point_cnt], 0);
//                     g_v_x = shot_point_vel_x[shot_point_cnt];
//                     g_v_y = shot_point_vel_y[shot_point_cnt];

//                     start = std::chrono::system_clock::now();
//                     continue;
//             }
//             // if (now - start > std::chrono::seconds(1))
//             if (now - start > std::chrono::duration<double>(shot_point_time[shot_point_cnt]))
//             { // send_velocity_command(0, 0, 0);
//                 RCLCPP_INFO(this->get_logger(), "前往下一点");
//                 // surround_shot_goto_next(dx_shot, dy_shot, shot_length, shot_width);

                
//                 shot_point_cnt++;
//                 // send_velocity_command(shot_point_vel_x[shot_point_cnt], shot_point_vel_y[shot_point_cnt], 0);
//                 g_v_x = shot_point_vel_x[shot_point_cnt];
//                 g_v_y = shot_point_vel_y[shot_point_cnt];
//                 //计算这条边走完应该到达什么位置
//                 shot_point_real_x += shot_point_time[shot_point_cnt-1] * shot_point_vel_x[shot_point_cnt-1];
//                 shot_point_real_y += shot_point_time[shot_point_cnt-1] * shot_point_vel_y[shot_point_cnt-1];
//                 start = std::chrono::system_clock::now();
//                 if (shot_point_cnt > 10)
//                 {
//                     break;
//                 }
//             }
//             send_velocity_command(g_v_x, g_v_y, 0);
//         }
//         else
//         {
//             pid_lost = 1;
//             RCLCPP_INFO(this->get_logger(), "看见桶了，执行PID");
//             PID(this->yolo->get_x(), this->yolo->get_y(), this->yolo->get_halt(), target_x, target_y, target_z, accuracy, z_accuracy, k, kp, ki, kd, dt);
//         }
//     }
//     send_velocity_command(0, 0, 0);
//     RCLCPP_INFO(this->get_logger(), "投弹!!投弹!!");
//     servo_controller(12, 1800);
//     rclcpp::sleep_for(std::chrono::seconds(1));
// }

void OffboardControl::Doshot()
{
    RCLCPP_INFO(this->get_logger(), "Doshot");
    auto Doshot_start = std::chrono::system_clock::now();
    auto start = std::chrono::system_clock::now();
    int surround_shot_count = 0;
    while (this->yolo->get_flag() == 0 || this->yolo->get_halt() > 1.6)
    {
        // RCLCPP_INFO(this->get_logger(), "当前高度：%lf",this->yolo->get_halt());
        auto now = std::chrono::system_clock::now();
        if (now - Doshot_start > std::chrono::seconds(60))
        {
            RCLCPP_INFO(this->get_logger(), "超时");
            break;
        }
        if (this->yolo->get_x1() == 0 && this->yolo->get_y1() == 0)
        {

            if (now - start > std::chrono::seconds(3))
            { // send_velocity_command(0, 0, 0);
                if (surround_shot_count > 13)
                {
                    break;
                }
                RCLCPP_INFO(this->get_logger(), "前往下一点");
                surround_shot_goto_next(dx_shot, dy_shot, shot_length, shot_width);
                start = std::chrono::system_clock::now();
                surround_shot_count++;
                
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "看见桶了，执行PID");
            if(this->yolo->get_halt()>target_z+0.4){
                PID(this->yolo->get_x1(), this->yolo->get_y1(), this->yolo->get_halt(), target_x, target_y, target_z, accuracy, z_accuracy, k1, kp, ki, kd, dt);
            }
            else{
                PID(this->yolo->get_x1(), this->yolo->get_y1(), this->yolo->get_halt(), target_x, target_y, target_z, accuracy, z_accuracy, k, kp, ki, kd, dt);
            }
            
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
    if (surround_shot_cnt > 13)
    {
        RCLCPP_INFO(this->get_logger(), "投弹区与已经全部遍历");
        return;
    }
    double x_shot = 0.0, y_shot = 0.0, angle = headingangle_compass;
    dxyToGlobal(x + (length * surround_shot_points[surround_shot_cnt].dx), y + (width * surround_shot_points[surround_shot_cnt].dy), headingangle_compass, x_shot, y_shot, angle);
    RCLCPP_INFO(this->get_logger(), "投弹区点位1 x: %lf   y: %lf    angle: %lf", x_shot, y_shot, angle);
    send_local_setpoint_command(x_shot, y_shot, shot_halt, angle);
    // rclcpp::sleep_for(std::chrono::seconds(3));

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
    double per_time = 4;

    dxyToGlobal(x, y + width, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位1_2 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(per_time)));

    dxyToGlobal(x - (length / 2.0), y, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位2_2 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(per_time)));

    dxyToGlobal(x - (length / 2.0), y + width, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位3_2 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(per_time)));
    
    dxyToGlobal(x + (length / 2.0), y, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位4_2 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(per_time*1.5)));
    
    dxyToGlobal(x + (length / 2.0), y + width, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位5_2 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(per_time)));
    
    dxyToGlobal(x, y, headingangle_compass, x_see, y_see, angle);
    RCLCPP_INFO(this->get_logger(), "侦查区点位6_2 x: %lf   y: %lf    angle: %lf", x_see, y_see, angle);
    send_local_setpoint_command(x_see, y_see, see_halt, angle);
}

void OffboardControl::Doland()
{
    RCLCPP_INFO(this->get_logger(), "Doland");
    double x_home = 0.0, y_home = 0.0, angle = headingangle_compass;
    dxyToGlobal(0, 1, headingangle_compass, x_home, y_home, angle);
    RCLCPP_INFO(this->get_logger(), "返回降落准备点 x: %lf   y: %lf    angle: %lf", x_home, y_home, angle);
    send_local_setpoint_command(x_home, y_home, 4, angle);
    rclcpp::sleep_for(std::chrono::seconds(6));

    dxyToGlobal(0, 0.3, headingangle_compass, x_home, y_home, angle);
    RCLCPP_INFO(this->get_logger(), "返回降落点 x: %lf   y: %lf    angle: %lf", x_home, y_home, angle);
    send_local_setpoint_command(x_home, y_home, 4, angle);
    rclcpp::sleep_for(std::chrono::seconds(2));
    auto Doland_start = std::chrono::system_clock::now();
    auto start = std::chrono::system_clock::now();
    bool is_land = false;
    static int surround_land = -3;
    while (!is_land)
    {
        auto now = std::chrono::system_clock::now();
        if (now - Doland_start > std::chrono::seconds(20) || surround_land > 3)
        {
            break;
        }
        if (this->yolo->get_x2() == 0 && this->yolo->get_y2() == 0)
        {
            if (now - start > std::chrono::duration<double>(1.5))
            {
                RCLCPP_INFO(this->get_logger(), "surround_land = %d", surround_land);

                dxyToGlobal(surround_land, 0, headingangle_compass, x_home, y_home, angle);
                RCLCPP_INFO(this->get_logger(), "land点 x: %lf   y: %lf    angle: %lf", x_home, y_home, angle);
                send_local_setpoint_command(x_home, y_home, shot_halt, angle);
                start = std::chrono::system_clock::now();
                surround_land++;
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "看见H了，执行PID_rtl");
            PID_rtl(this->yolo->get_x2(), this->yolo->get_y2(), now_halt, target_x, target_y, is_land);
        }
    }
    send_velocity_command_with_time(0, 0, 0, 1);
    RCLCPP_INFO(this->get_logger(), "1s后降落");
    set_mode("LAND");
}