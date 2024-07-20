#include <iostream>
#include <string>
#include <chrono>
#include <memory>
#include <thread>
#include <fstream>
#include <vector>
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
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/command_code.hpp>
#include "sensor_msgs/msg/range.hpp"
#include <cmath>
#include "YOLO.hpp"

#define PI 3.14159265358979323846

class OffboardControl : public rclcpp::Node
{
public:
    OffboardControl(std::shared_ptr<YOLO> yolo);
    // 初始化运动节点
    void init();
    // 起飞函数s
    void takeoff(int halt);
    // 控制线速度
    void send_velocity_command(double linear_x, double linear_y, double linear_z);
    // 控制线速度with time
    void send_velocity_command_with_time(double linear_x, double linear_y, double linear_z, int time);
    // 控制位置
    void send_local_setpoint_command(double x, double y, double z, double angle);
    // 坐标转换函数
    void dxyToGlobal(double dx, double dy, double headingangle_compass, double &x, double &y, double &angle);
    mavros_msgs::msg::State current_state;
    // 定时器回调函数
    void run();
    // void timer_callback(void);

    // 设置模式函数
    void set_mode(const std::string &mymode);
    // 油门锁解锁函数
    void arm();
    // 设置家地址
    void set_home_position();

    // 舵机控制函数
    void servo_controller(int servo_number, float position);
    // PID控制函数
    void PID(double now_x, double now_y, double now_z, double target_x, double target_y, double target_z, double accuracy, double z_accuracy, double k, double kp, double ki, double kd, double dt);

    // 投弹函数
    void Doshot();
    //环绕投弹区
    void surround_shot_goto_next(double x, double y, double length, double width);
    // 环绕侦查区
    void surround_see(double x, double y, double length, double width);

    // 降落时的PID控制函数
    void PID_rtl(double now_x, double now_y, double now_z, double target_x, double target_y, bool &is_land);
    // 降落函数
    void Doland();

private:
    typedef struct
    {
        double x;
        double y;
        double z;
    } Location;
    Location home_position;
    double now_halt; // 无人机当前高度，激光雷达数据
    std::shared_ptr<YOLO> yolo;
    double headingangle_compass = 180.0;
    const double shot_length = 8.0;
    const double shot_width = 5.0;
    const double shot_halt = 4.0;
    const double see_length = 7.0;
    const double see_width = 6.0;
    const double see_halt = 3.0;
    const int servo_number = 12;
    const double dx_shot = 0.0, dy_shot = 10.0;
    const double dx_see = 0.0, dy_see = 15;//53.7
    double target_x = 0.0;
    double target_y = 0.0;
    const double target_z = 1.2;
    const double accuracy = 20.0;
    const double z_accuracy = 0.1;

    const double k = 0.002;
    const double dt = 0.5;
    double kp = 0.0; // 0.48
    double ki = 0.0; // 0.01
    double kd = 0.0; // 0.45

    const double max_vx = 0.6;
    const double max_vy = 0.6;
    const double max_vz = 0.6;
    struct surround_shot_coord
    {
        double dx, dy;
    } surround_shot_points[13] = {{0.0, 0.0}, {0.0, 1.0}, {-0.16667, 0.66667}, {-0.16667, 0.33333}, {0.0, 0.0}, {0.16667, 0.33333}, {0.16667, 0.66667}, {0.0, 1.0}, {-0.33333, 1.0}, {-0.33333, 0.0}, {0.33333, 0.0}, {0.33333, 1.0}, {0.0, 1.0}};
    // auto node = rclcpp::Node::make_shared("offboard");
    // rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_publisher;
    rclcpp::Subscription<mavros_msgs::msg::HomePosition>::SharedPtr home_position_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangefinder_sub; // 激光雷达高度
    rclcpp::Client<mavros_msgs::srv::CommandLong>::SharedPtr servo_client_;

    void range_callback(const sensor_msgs::msg::Range::SharedPtr msg);
    void state_callback(const mavros_msgs::msg::State::SharedPtr msg);
    void home_position_callback(const mavros_msgs::msg::HomePosition::SharedPtr msg);

    // geometry_msgs::msg::PoseStamped last_pose_;
    // rclcpp::Time last_update_time_;
    // 无人机状态回调函数

    // 声明定时器
    rclcpp::TimerBase::SharedPtr timer_;
};