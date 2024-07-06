#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/msg/state.hpp>
#include "mavros_msgs/msg/global_position_target.hpp"


mavros_msgs::msg::State current_state;

// 无人机状态回调函数
void state_callback(const mavros_msgs::msg::State::SharedPtr msg) {
    current_state = *msg;
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("offboard");

    auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
        "/mavros/state", 10, state_callback);

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

    // 发布起飞的指令，向上飞行5米
    auto takeoff_cl = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    auto srv_takeoff = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    srv_takeoff->altitude = 5;

    
    srv_takeoff->min_pitch = 0;
    srv_takeoff->yaw = 0;

    auto takeoff_result = takeoff_cl->async_send_request(srv_takeoff);
    if (rclcpp::spin_until_future_complete(node, takeoff_result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(node->get_logger(), "Takeoff send ok");
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed Takeoff");
    }

    // 等待10秒
    rclcpp::sleep_for(std::chrono::seconds(10));

    // auto local_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>(
    //     "mavros/setpoint_velocity/cmd_vel_unstamped", 10);
    auto local_pos_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    
    auto time_start = node->now();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.z = 2;

    geometry_msgs::msg::Twist vel;

    // 转圈圈飞行
    while (rclcpp::ok()) {
        auto now = node->now();
        auto elapsed = (now - time_start).seconds();
        if(elapsed<10){
            pose.pose.position.x = 10;
            pose.pose.position.y = 0;
        }
        else{
            pose.pose.position.x = 5;
            pose.pose.position.y = 10;
        }
        

        // vel.linear.x = -1;
        // vel.linear.y = 0;


        local_pos_pub->publish(pose);
        RCLCPP_INFO(node->get_logger(), "Pub pose: x=%d,y=%d",pose.pose.position.x,pose.pose.position.y);
        // rclcpp::sleep_for(std::chrono::seconds(10)); // 暂停1秒
        // local_vel_pub->publish(vel);
        // RCLCPP_INFO(node->get_logger(), "Pub vel");

        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
