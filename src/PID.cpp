#include "OffboardControl.hpp"

void OffboardControl::PID(double now_x, double now_y, double now_z, double target_x, double target_y, double target_z, double accuracy,double z_accuracy, double k, double kp, double ki, double kd,double dt){
    RCLCPP_INFO(this->get_logger(), "Publishing setpoint: x=%lf, y=%lf, z=%lf", now_x, now_y, now_z);
	static double previous_error_x = target_x-now_x;
    static double previous_error_y = target_y-now_y;
    static double previous_error_z = target_z-now_z;

    static double integral_x = 0;
    static double integral_y = 0;
    static double integral_z = 0;

	double error_x = target_x-now_x;
	double error_y = target_y-now_y;
	double error_z = target_z-now_z;
 
	const static int n = 10;
	const static double integral_limit = 0.1;
	static double integral_[n][4] = {0}; // 0:x, 1:y, 2:z
	static int i = 0;
	// 移除旧的积分值
	integral_x -= integral_[i][0];
	integral_y -= integral_[i][1];
	integral_z -= integral_[i][2];
	// 添加新的积分值
	integral_[i][0] = error_x * dt;
	integral_[i][1] = error_y * dt;
	integral_[i][2] = error_z * dt;
	// 更新积分，并引入积分限幅
	integral_x = std::min(std::max(integral_x + error_x * dt, -integral_limit), integral_limit);
	integral_y = std::min(std::max(integral_y + error_y * dt, -integral_limit), integral_limit);
	integral_z = std::min(std::max(integral_z + error_z * dt, -integral_limit), integral_limit);
	i = (i+1)%n;


	double velocity_x = (error_x - previous_error_x) / dt;
	double velocity_y = (error_y - previous_error_y) / dt;
	double velocity_z = (error_z - previous_error_z) / dt;

	//double derivative_x = (error_x - previous_error_x) / dt;
	//double derivative_y = (error_y - previous_error_y) / dt;
	//double derivative_z = (error_z - previous_error_z) / dt;
	//double derivative_yaw = (error_yaw - previous_error_yaw) / dt;	

	double output_x = kp * error_x + ki * integral_x + kd * velocity_x;
	double output_y = kp * error_y + ki * integral_y + kd * velocity_y;
	double output_z = kp * error_z + ki * integral_z + kd * velocity_z;

	previous_error_x = error_x;
	previous_error_y = error_y;
	previous_error_z = error_z;

	/*
	if(abs(output_x)>max_vx||abs(output_y)>max_vy){
		double rx= abs(output_x)/max_vx;
		double ry= abs(output_y)/max_vy;
		if(rx>ry){
			output_x*=rx;
			output_y*=rx;
		}else{
			output_x*=ry;
			output_y*=ry;
		}
	}*/
	double output_xy_d = sqrt(pow(output_x,2)+pow(output_y,2));
	//double output_xy_d = output_x + output_y;
	const double max_vxy = sqrt(pow(max_vx,2)+pow(max_vy,2));
	//const double max_vxy = max_vx + max_vy;
	if(output_xy_d>max_vxy){
		double r= max_vxy/output_xy_d;
		output_x*=r;
		output_y*=r;
	}
	if (output_x > max_vx) output_x = max_vx;
	if (output_x < -max_vx) output_x = -max_vx;
	if (output_y > max_vy) output_y = max_vy;
	if (output_y < -max_vy) output_y = -max_vy;
	if (output_z > max_vz) output_z = max_vz;
	if (output_z < -max_vz) output_z = -max_vz;
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: error: x=%f, y=%f, z=%f", error_x, error_y,error_z);
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: integral: x=%f, y=%f, z=%f", integral_x, integral_y, integral_z);
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: velocity: x=%f, y=%f, z=%f", velocity_x, velocity_y, velocity_z);
	RCLCPP_INFO(this->get_logger(), "publish_trajectory_setpoint: output: x=%f, y=%f, z=%f", output_x, output_y, output_z);
    send_velocity_command(output_x, output_y, output_z);
	// if(at_check_point()){
 	// 	//RCLCPP_INFO(this->get_logger(), "at_check_point");
	// 	previous_error_x = 0;
	// 	previous_error_y = 0;
	// 	previous_error_z = 0;
	// 	previous_error_yaw = 0;
	// 	integral_x = 0;
	// 	integral_y = 0;
	// 	integral_z = 0;
	// 	integral_yaw = 0;
	// }
    if(abs(now_x-target_x)<accuracy && abs(now_y-target_y)<accuracy && abs(now_z-target_z)<z_accuracy){
        
 		//RCLCPP_INFO(this->get_logger(), "at_check_point");
		previous_error_x = 0;
		previous_error_y = 0;
		previous_error_z = 0;
		integral_x = 0;
		integral_y = 0;
		integral_z = 0;
        // return true;
	}
    // return false;
}
    