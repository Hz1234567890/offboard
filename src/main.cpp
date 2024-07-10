#include "OffboardControl.hpp"

// // 无人机状态回调函数
// mavros_msgs::msg::State current_state;
// void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
//     current_state = *msg;
// }
int main(int argc, char **argv) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor executor;

    auto yolo=std::make_shared<YOLO>();
    auto node = std::make_shared<OffboardControl>(yolo);

    rclcpp::Rate rate(1);  // 1 Hz
    while(rclcpp::ok() && !node->current_state.connected){
        RCLCPP_INFO(node->get_logger(), "Waiting for current_state to connect...");
        rclcpp::spin_some(node);
        rate.sleep();
    }

    executor.add_node(yolo);
    executor.add_node(node);

    executor.spin();

    node->run();
    rclcpp::shutdown();
    return 0;
}

