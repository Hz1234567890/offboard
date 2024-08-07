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
            std::bind(&YOLO::coord_callback, this, std::placeholders::_1));
        rangefinder_sub = this->create_subscription<sensor_msgs::msg::Range>("/mavros/rangefinder/rangefinder", 10,
                                                                             std::bind(&YOLO::range_callback, this, std::placeholders::_1));
    }
    float get_x1()
    {
        return x1;
    }
    float get_y1()
    {
        return y1;
    }

    float get_x2()
    {
        return x2;
    }
    float get_y2()
    {
        return y2;
    }

    int get_flag()
    {
        return flag_servo;
    }
    double get_halt()
    {
        return halt;
    }

private:
    float x1;//x1和y1是圆坐标
    float y1;
    float x2;//x2和y2是H坐标
    float y2;
    int flag_servo;
    double halt;
    rclcpp::Subscription<ros2_interfaces::msg::Coord>::SharedPtr subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr rangefinder_sub; // 激光雷达高度
    void coord_callback(const ros2_interfaces::msg::Coord::SharedPtr msg)
    {
        x1 = msg->x1;
        y1 = msg->y1;
        x2 = msg->x2;
        y2 = msg->y2;
        flag_servo = msg->flag_servo;
        // (void)flag;
        // RCLCPP_INFO(this->get_logger(), "收到坐标(%f, %f), flag_servo = %d", x, y, flag);
    }
    void range_callback(const sensor_msgs::msg::Range::SharedPtr msg)
    {
        halt = msg->range;
    }
};
