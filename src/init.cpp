#include <iostream>  
#include <string>  

#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>


//初始化运动节点
void OffboardControl::init(int argc, char **argv){
    //创建一个Rate对象，设置为每秒20次循环
    rclcpp::Rate rate(20);

    RCLCPP_INFO(node->get_logger(), "Initializing...");
    while (rclcpp::ok() && !current_state.connected) {
        rclcpp::spin_some(node);
        rate.sleep();
    }
    RCLCPP_INFO(node->get_logger(), "Connected.");

    // 设置无人机模式为GUIDED
    auto cl = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    auto srv_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    srv_set_mode->base_mode = 0;
    srv_set_mode->custom_mode = "GUIDED";

    auto result = cl->async_send_request(srv_set_mode);
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "SetMode send ok");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed SetMode");
        return -1;
    }

    //按下回车键以解锁无人机
    
    std::string key;  
    while (true) {  
        RCLCPP_INFO(node->get_logger(), "解锁前所有准备已完成，按下回车解锁无人机");
        // 读取一整行输入  
        std::getline(std::cin, key);  

        // 检查输入是否为空（即用户只按下了回车键）  
        if (key.empty()) {  
            break; // 跳出循环  
        }  
    }  

    // 解锁无人机
    auto arming_cl = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    auto srv_arm = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    srv_arm->value = true;

    auto arm_result = arming_cl->async_send_request(srv_arm);
    if (rclcpp::spin_until_future_complete(node, arm_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "ARM send ok");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed arming");
    }
  

}