#include <mavros_msgs/srv/command_bool.hpp>

void OffboardControl::takeoff(int halt){
    // 发布起飞的指令，向上飞行10米
    auto takeoff_cl = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    auto srv_takeoff = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    srv_takeoff->altitude = halt;
    auto takeoff_result = takeoff_cl->async_send_request(srv_takeoff);
    if (rclcpp::spin_until_future_complete(node, takeoff_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Takeoff send ok");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed Takeoff");
    }
}