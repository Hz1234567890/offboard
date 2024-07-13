#include "ros2_interfaces/msg/coord.hpp"
// #include <opencv2/opencv.hpp>
// #include "cv_bridge/cv_bridge.h"

class YOLO : public rclcpp::Node
{
public:
    YOLO() : Node("image_pub")
    {
        subscriber_ = this->create_subscription<ros2_interfaces::msg::Coord>(
            "coord", 
            10, 
            std::bind(&YOLO::coord_callback, this, std::placeholders::_1)
        );
    }
	float get_x(){
		return x;
	}
	float get_y(){
		return y;
	}
    int get_flag(){
        return flag_servo;
    }
private:
	float x;
	float y;
    int flag_servo;
    rclcpp::Subscription<ros2_interfaces::msg::Coord>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    void coord_callback(const ros2_interfaces::msg::Coord::SharedPtr msg)
    {
         x = msg->x;
         y = msg->y;
        flag_servo = msg->flag_servo;
		// (void)flag;
        //RCLCPP_INFO(this->get_logger(), "收到坐标(%f, %f), flag_servo = %d", x, y, flag);
    }
    
};
