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
        rangefinder_sub = this->create_subscription<sensor_msgs::msg::Range>("/mavros/rangefinder/rangefinder", 10, 
                                                                    std::bind(&YOLO::range_callback, this, std::placeholders::_1));
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
    double get_halt(){
        return halt;
    }
private:
	float x;
	float y;
    int flag_servo;
    double halt;
    rclcpp::Subscription<ros2_interfaces::msg::Coord>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangefinder_sub; // 激光雷达高度
    void coord_callback(const ros2_interfaces::msg::Coord::SharedPtr msg)
    {
         x = msg->x;
         y = msg->y;
        flag_servo = msg->flag_servo;
		// (void)flag;
        //RCLCPP_INFO(this->get_logger(), "收到坐标(%f, %f), flag_servo = %d", x, y, flag);
    }
    void range_callback(const sensor_msgs::msg::Range::SharedPtr msg){
        halt=msg->range;
    }
};
