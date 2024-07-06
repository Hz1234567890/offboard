#include "main.hpp"

void OffboardControl::timer_callback(){
    init(argc, argv);
    takeoff(3);//起飞高度3米    
    send_local_setpoint_command(10,20,3,90);
    send_velocity_command(2,0,0);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OffboardControl>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

