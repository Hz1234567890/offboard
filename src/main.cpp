#include "OffboardControl.hpp"

// // 无人机状态回调函数
// mavros_msgs::msg::State current_state;
// void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
//     current_state = *msg;
// }
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControl>();

    rclcpp::Rate rate(1);  // 1 Hz
    while(rclcpp::ok() && !node->current_state.connected){
        RCLCPP_INFO(node->get_logger(), "Waiting for current_state to connect...");
        rclcpp::spin_some(node);
        rate.sleep();
    }
    node->run();
    rclcpp::shutdown();
    return 0;
}

