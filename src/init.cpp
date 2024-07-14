#include "OffboardControl.hpp"

// // 无人机状态回调函数
// void OffboardControl::state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
//     current_state = *msg;
// }

// 初始化运动节点
void OffboardControl::init()
{
    RCLCPP_INFO(this->get_logger(), "获取PID参数");

    std::string filePath = "/home/hz/ros2_ws/src/offboard/src/can.txt";
    std::ifstream file(filePath);
    if (!file.is_open())
    {
        RCLCPP_ERROR(this->get_logger(), "无法打开文件");
    }

    std::vector<double> parameters;
    double number;
    while (file >> number)
    {
        parameters.push_back(number);
    }
    file.close();

    // 将vector中的数值复制到数组中
    int size = parameters.size();
    double *array = new double[size];
    for (int i = 0; i < size; ++i)
    {
        array[i] = parameters[i];
    }
    kp = array[0];
    ki = array[1];
    kd = array[2];
    RCLCPP_INFO(this->get_logger(), "kp:%lf ki:%lf  kd:%lf", kp, ki, kd);
    RCLCPP_INFO(this->get_logger(), "开始初始化舵机");
    // 初始化舵机操作
    while (!servo_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the Servo service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Servo Service not available, waiting again...");
    }

    servo_controller(12, 1050);
    RCLCPP_INFO(this->get_logger(), "结束初始化舵机");
    // 创建一个Rate对象，设置为每秒20次循环
    rclcpp::Rate rate(10);

    // 重新设置家地址
    set_home_position();

    RCLCPP_INFO(this->get_logger(), "Initializing...");
    // 设置无人机模式为GUIDED
    set_mode("GUIDED");

    std::string key;
    while (true)
    {
        RCLCPP_INFO(this->get_logger(), "解锁前所有准备已完成，按下回车解锁无人机");
        // 读取一整行输入
        std::getline(std::cin, key);

        // 检查输入是否为空（即用户只按下了回车键）
        if (key.empty())
        {
            RCLCPP_INFO(this->get_logger(), "开始解锁");
            break; // 跳出循环
        }
    }

    arm();
}

// setMode
void OffboardControl::set_mode(const std::string &mymode)
{
    auto cl = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    auto srv_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    srv_set_mode->base_mode = 0;
    srv_set_mode->custom_mode = mymode;

    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "SetMode Command send");
    auto result = cl->async_send_request(srv_set_mode);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(get_logger(), mymode.c_str(), "mode set successfully");
    }
    else
    {
        RCLCPP_ERROR(get_logger(), "Failed to set", mymode.c_str(), " mode");
    }
}

// 油门锁解锁函数
void OffboardControl::arm()
{
    // 解锁无人机
    auto arming_cl = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    auto srv_arm = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    srv_arm->value = true;

    while (!arming_cl->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    RCLCPP_INFO(this->get_logger(), "Arm Command send");
    auto arm_result = arming_cl->async_send_request(srv_arm);
    rclcpp::sleep_for(std::chrono::seconds(3));
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), arm_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Arm success");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Arm Failed");
    }
}

// 舵机控制函数
void OffboardControl::servo_controller(int servo_number, float position)
{
    auto servo_request = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    servo_request->command = mavros_msgs::msg::CommandCode::DO_SET_SERVO;
    servo_request->param1 = servo_number;
    servo_request->param2 = position;

    auto servo_result = servo_client_->async_send_request(servo_request);

    // Wait for the result
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), servo_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Servo set successfully");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service set_servo");
    }
}

// 设置无人机家的位置
// /mavros/cmd/set_home [mavros_msgs/srv/CommandHome]
void OffboardControl::set_home_position()
{
    rclcpp::Client<mavros_msgs::srv::CommandHome>::SharedPtr set_home_client_;
    set_home_client_ = this->create_client<mavros_msgs::srv::CommandHome>("mavros/cmd/set_home");
    auto srv_home = std::make_shared<mavros_msgs::srv::CommandHome::Request>();
    srv_home->current_gps = true;

    while (!set_home_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    RCLCPP_INFO(this->get_logger(), "set home command send");
    auto home_result = set_home_client_->async_send_request(srv_home);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), home_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "Set Home success");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Set Home Failed");
    }
}