#include "OffboardControl.hpp"

void OffboardControl::takeoff(int halt){
    // 发布起飞的指令，向上飞行10米
    RCLCPP_INFO(this->get_logger(), "开始发送takeoff指令");
    auto takeoff_cl = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    auto srv_takeoff = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    srv_takeoff->altitude = halt;
    auto takeoff_result = takeoff_cl->async_send_request(srv_takeoff,
        [this](rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedFuture future) {
			auto status = future.wait_for(std::chrono::seconds(1));
			if (status == std::future_status::ready) {
				auto reply = future.get()->success;
				RCLCPP_INFO(this->get_logger(), "TakeOff: %s", reply ? "success" : "failed");
			} else {
				// Wait for the result.
				RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
			}
		});
    // if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), takeoff_result) ==
    //     rclcpp::FutureReturnCode::SUCCESS)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Takeoff send ok");
    // } else {
    //     RCLCPP_ERROR(this->get_logger(), "Failed Takeoff");
    // }
}