#include "OffboardControl.hpp"

// // 无人机状态回调函数
// void OffboardControl::state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
//     current_state = *msg;
// }

//初始化运动节点
void OffboardControl::init(){
    //创建一个Rate对象，设置为每秒20次循环
    rclcpp::Rate rate(10);

    //重新设置家地址

    RCLCPP_INFO(this->get_logger(), "Initializing...");
    // 设置无人机模式为GUIDED
    auto cl = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    auto srv_set_mode = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    srv_set_mode->base_mode = 0;
    srv_set_mode->custom_mode = "GUIDED";

    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(this->get_logger(), "SetMode Command send");
    auto result = cl->async_send_request(srv_set_mode);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(get_logger(), "GUIEDE mode set successfully");
    } else {
        RCLCPP_ERROR(get_logger(), "Failed to set GUIDED mode");
    }
    // auto result_future = cl->async_send_request(srv_set_mode,
    //     [this](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture future) {
	// 		auto status = future.wait_for(std::chrono::seconds(1));
	// 		if (status == std::future_status::ready) {
	// 			auto reply = future.get()->mode_sent;
	// 			RCLCPP_INFO(this->get_logger(), "Mode switch: %s", reply ? "success" : "failed");
	// 		} 
    //         else {
	// 			// Wait for the result.
	// 			RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
	// 		}
    //         RCLCPP_INFO(this->get_logger(), "SetEnd");
	// 	});

    // // 在节点中使用 spin_until_future_complete 等待 future 完成
    // rclcpp::spin_until_future_complete(this->get_node_base_interface(), result);

    // // 处理异步操作完成后的逻辑
    // if (result.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    //     auto response = result.get();
    //     if (response->mode_sent) {
    //         RCLCPP_INFO(get_logger(), "GUIEDE mode set successfully");
    //     } else {
    //         RCLCPP_ERROR(get_logger(), "Failed to set GUIDED mode");
    //     }
    // } else {
    //     RCLCPP_ERROR(get_logger(), "Service call timed out");
    // }
    
    //按下回车键以解锁无人机
    
    std::string key;  
    while (true) {  
        RCLCPP_INFO(this->get_logger(), "解锁前所有准备已完成，按下回车解锁无人机");
        // 读取一整行输入  
        std::getline(std::cin, key);  

        // 检查输入是否为空（即用户只按下了回车键）  
        if (key.empty()) {  
            RCLCPP_INFO(this->get_logger(), "开始解锁");
            break; // 跳出循环  
        }  
    }  

    // 解锁无人机
    auto arming_cl = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    auto srv_arm = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    srv_arm->value = true;

    while (!arming_cl->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
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
    } else {
        RCLCPP_ERROR(this->get_logger(), "Arm Failed");
    }
    // auto arm_result = arming_cl->async_send_request(srv_arm,
    //     [this](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture future) {
	// 		auto status = future.wait_for(std::chrono::seconds(1));
	// 		if (status == std::future_status::ready) {
	// 			auto reply = future.get()->success;
	// 			RCLCPP_INFO(this->get_logger(), "Arm motors: %s", reply ? "success" : "failed");
	// 		} else {
	// 			// Wait for the result.
	// 			RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
	// 		}
	// 	});

    }