//时间同步加低通滤波器消除抖动

#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <array>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "ibvs_proj/img_moment.h"

//定义惯性环节类函数
class InertialElement {
private:
    double T;  // Time constant
    std::array<double, 3> y_prev; // Previous output for 3 dimensions

public:
    InertialElement(double timeConstant) 
        : T(timeConstant), y_prev({0.0, 0.0, 0.0}) {}

    std::array<double, 3> update(const std::array<double, 3>& input, double dt) {
        // Discrete transfer function coefficients
        double a = T / dt;
        double b = 1 / (a + 1);
        
        // Update equation for each dimension
        std::array<double, 3> y;
        for (int i = 0; i < 3; ++i) {
            y[i] = b * input[i] + (a * b) * y_prev[i];
            y_prev[i] = y[i]; // Store current output for next iteration
        }
        
        return y;
    }
};


// 定义一个通用的低通滤波器类
class LowPassFilter {
private:
    double T;  // 时间常数
    std::array<double, 3> y_prev; // 上次输出

public:
    LowPassFilter(double timeConstant) 
        : T(timeConstant), y_prev({0.0, 0.0, 0.0}) {}

    std::array<double, 3> update(const std::array<double, 3>& input, double dt) {
        double alpha = dt / (T + dt);
        for (int i = 0; i < 3; ++i) {
            y_prev[i] = alpha * input[i] + (1 - alpha) * y_prev[i];
        }
        return y_prev;
    }
};




//定义观测器类函数
class Observer {
private:

    double time0;
    double time;

    ros::Time last_call_time_; // 上次调用的时间戳

    std::array<double,3> image_q0; //初始的图像矩计算
    bool target_image_received_; //目标点数据接收标志
    bool firstimage; //第一次接收图像标志

    double T1;
    double T2;
    double dt;

    ros::NodeHandle nh_;
    ros::Publisher target_est_vel_pub_;

    double yaw;
    double yaw_rate;
    double velocity_x;
    double velocity_y;
    double velocity_z;

    std::array<double,3> image_q; //图像矩的实时

    double q_yaw;

    InertialElement velocity_x_filter;
    InertialElement qx_filter;

    // 新增的低通滤波器实例，用于平滑最终的速度估计
    LowPassFilter velocity_est_filter;

    // 创建订阅者
    message_filters::Subscriber<ibvs_proj::img_moment> image_sub_;
    message_filters::Subscriber<geometry_msgs::TwistStamped> velocity_sub_;

    // 创建同步器
    typedef message_filters::sync_policies::ApproximateTime<ibvs_proj::img_moment, geometry_msgs::TwistStamped> MySyncPolicy;
    std::unique_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

public:
    // 初始化类函数
    Observer(ros::NodeHandle &nh) : nh_(nh), T1(0.01), T2(0.01),
        velocity_x_filter(T1), qx_filter(T2), velocity_est_filter(0.5) {

        // 订阅图像矩信息和速度信息
        image_sub_.subscribe(nh_, "target_image", 30);
        velocity_sub_.subscribe(nh_, "mavros/local_position/velocity_body", 30);

        // 发布目标速度估计信息
        target_est_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("target_est_vel_velocity", 30);

        // 初始化同步器
        sync_.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), image_sub_, velocity_sub_));
        sync_->registerCallback(boost::bind(&Observer::syncCallback, this, _1, _2));

        // 初始化变量
        yaw = 0;
        yaw_rate = 0;
        velocity_x = 0;
        velocity_y = 0;
        velocity_z = 0;
        target_image_received_ = false;
        firstimage = false;

        image_q0 = {0.0, 0.0, 0.0};
        time0 = 0;
        time = 0;
        image_q = {0.0, 0.0, 0.0};

        last_call_time_ = ros::Time::now(); // 初始化上次调用时间为当前时间
    }

    void syncCallback(const ibvs_proj::img_moment::ConstPtr& img_msg,
                      const geometry_msgs::TwistStamped::ConstPtr& vel_msg) {
        // 更新图像和速度信息
        image_q[0] = img_msg->data[0];
        image_q[1] = img_msg->data[1];
        image_q[2] = img_msg->data[2];
        q_yaw = img_msg->data[3];
        target_image_received_ = true;

        velocity_x = vel_msg->twist.linear.x;
        velocity_y = vel_msg->twist.linear.y;
        velocity_z = vel_msg->twist.linear.z;
        yaw_rate = vel_msg->twist.angular.z;

        if (!firstimage) {
            image_q0[0] = image_q[0];
            image_q0[1] = image_q[1];
            image_q0[2] = image_q[2];

            time0 = ros::Time::now().toSec();
            ROS_INFO("First_imageOK");
            firstimage = true;
        }

        
        // 计算自上次调用以来的时间差
        ros::Time current_time = ros::Time::now();
        double dt = std::max((current_time - last_call_time_).toSec(), 0.0000000001);
        last_call_time_ = current_time;

        double t = ros::Time::now().toSec() - time0;
        std::array<double, 3> qx_filtered = qx_filter.update(image_q, dt);
        std::array<double, 3> hx = {-yaw_rate * image_q[1] + velocity_x,
                                    yaw_rate * image_q[0] + velocity_y,
                                    velocity_z};
        std::array<double, 3> hx_filtered = velocity_x_filter.update(hx, dt);

        // 构建并发布目标速度估计消息
        geometry_msgs::TwistStamped target_vel;
        //target_vel.header.stamp = vel_msg->header.stamp; // 使用速度消息的时间戳以保持同步
        target_vel.header.stamp = ros::Time::now();
        target_vel.twist.linear.x = -0.2*(hx_filtered[0] + 1 / T1 * (image_q[0] - qx_filtered[0] - exp(-1 / T1 * t) * (image_q0[0])));
        target_vel.twist.linear.y = -0.2*(hx_filtered[1] + 1 / T1 * (image_q[1] - qx_filtered[1] - exp(-1 / T1 * t) * (image_q0[1])));
        target_vel.twist.linear.z = -0.2*(hx_filtered[2] + 1 / T1 * (image_q[2] - qx_filtered[2] - exp(-1 / T1 * t) * (image_q0[2])));

        
         // 对目标速度估计进行低通滤波
        std::array<double, 3> final_velocity = velocity_est_filter.update(
            {target_vel.twist.linear.x, target_vel.twist.linear.y, target_vel.twist.linear.z}, dt);

        // 更新目标速度估计消息
        target_vel.twist.linear.x = final_velocity[0];
        target_vel.twist.linear.y = final_velocity[1];
        target_vel.twist.linear.z = final_velocity[2];
        
        ROS_INFO("est_Vx: %f, est_Vy: %f, dt: %f", target_vel.twist.linear.x, target_vel.twist.linear.y, dt);
        ROS_INFO("hs1: %f, hs2: %f", hx_filtered[0], hx_filtered[1]);
        ROS_INFO("qx1: %f, qx2: %f", qx_filtered[0], qx_filtered[1]);

        target_est_vel_pub_.publish(target_vel);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "observer");
    ros::NodeHandle nh;

    Observer observer(nh);
    ros::spin();

    return 0;
}