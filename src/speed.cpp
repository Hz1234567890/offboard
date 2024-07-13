#include "OffboardControl.hpp"

void OffboardControl::send_velocity_command(double linear_x, double linear_y, double linear_z){
    geometry_msgs::msg::TwistStamped twist_stamped;
    double x = 0.0, y = 0.0, angle = headingangle_compass;
    dxyToGlobal(linear_x, linear_y, headingangle_compass, x, y, angle);

    twist_stamped.header.stamp = this->now();
    twist_stamped.twist.linear.x = x;
    twist_stamped.twist.linear.y = y;
    twist_stamped.twist.linear.z = linear_z;
    twist_stamped_publisher->publish(twist_stamped);
}

void OffboardControl::send_velocity_command_with_time(double linear_x, double linear_y, double linear_z,int time){
    geometry_msgs::msg::TwistStamped twist_stamped;

    double x = 0.0, y = 0.0, angle = headingangle_compass;
    dxyToGlobal(linear_x, linear_y, headingangle_compass, x, y, angle);

    twist_stamped.header.stamp = this->now();
    twist_stamped.twist.linear.x = x;
    twist_stamped.twist.linear.y = y;
    twist_stamped.twist.linear.z = linear_z;

    twist_stamped_publisher->publish(twist_stamped);
    rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(time)));
    twist_stamped.twist.linear.x = 0;
    twist_stamped.twist.linear.y = 0;
    twist_stamped.twist.linear.z = 0;
    twist_stamped_publisher->publish(twist_stamped);
}

void OffboardControl::send_local_setpoint_command(double x, double y, double z,double angle){
    geometry_msgs::msg::PoseStamped msg;
    //时间戳等header信息
    msg.header.stamp = this->now();
    msg.header.frame_id = "base_link";
    //坐标位置 单位为m
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;
    //旋转四元数
    
    double radians_angle = angle * PI / 180.0;
    msg.pose.orientation.x = 0;
	msg.pose.orientation.y = 0;
	msg.pose.orientation.z = sin(radians_angle / 2);
	msg.pose.orientation.w = cos(radians_angle / 2);

    local_pos_pub->publish(msg);
}