#include <ros/ros.h>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "ibvs_proj/img_moment.h"

class IBVSController {
public:


    IBVSController(ros::NodeHandle &nh) : nh_(nh) {

        // 订阅目标点数据
        target_points_sub_ = nh_.subscribe<ibvs_proj::img_moment>("target_points", 30, &IBVSController::targetPointsCallback, this);

        // 发布控制速度
        control_vel_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("ibvs_control_vel", 30);

        // 发布图像矩
        image_pub_ = nh.advertise<ibvs_proj::img_moment>("target_image",30);
    }


    void targetPointsCallback(const ibvs_proj::img_moment::ConstPtr &msg) {
        // 获取目标点数据
        target_points_ = msg->data;
        // 创建速度消息
        geometry_msgs::TwistStamped control_vel;

        if (target_points_.size() != 9) { // 检查是否有有效的目标点数据
            control_vel.header.stamp = ros::Time::now();
            control_vel.twist.linear.x = 0;
            control_vel.twist.linear.y = 0;
            control_vel.twist.linear.z = 0;
            control_vel.twist.angular.z = 0;

            control_vel_pub_.publish(control_vel);
            ROS_INFO("Invalid target points received.");
            return;
        }
        // 相机焦距
        float f = 277.191356;
        float ad = 17000;
        // 处理目标点数据，生成目标图像据
        float ug, ng, u11, u20, u02, a;
        ug = ng = u11 = u20 = u02 = a = 0;
        for (int i = 0; i < 8; i += 2) {
            ug += target_points_[i] - 960;
            ng += target_points_[i + 1] - 540;
        }
        ug *= 0.25; ng *= 0.25;

        for (int i = 0; i < 8; i += 2) {
            u11 += (target_points_[i] - 960 - ug) * (target_points_[i + 1] - 540 - ng);
            u20 += pow((target_points_[i] - 960 - ug), 2);
            u02 += pow((target_points_[i + 1] - 540 - ng), 2);
        }

        a = u02 + u20;
        std::cout<<"a="<<a<<std::endl;;

        ibvs_proj::img_moment target_msg;
        float qx, qy, qz, q_yaw;
        qz = 7 * sqrt(ad / a);
        qx = qz * (ng / f);
        qy = qz * (ug / f);
        q_yaw = 0.5 * atan2(2 * u11, u02 - u20); // 使用atan2以获得更好的角度计算

        // 发布图像矩
        target_msg.data.push_back(qx);
        target_msg.data.push_back(qy);
        target_msg.data.push_back(qz);
        target_msg.data.push_back(q_yaw);
        target_msg.header = msg->header; // 使用原始消息的时间戳保持同步
        image_pub_.publish(target_msg);
        ROS_INFO("image_pub.done");

        // 创建速度系数
        double lambda = -0.08;

        // 创建向量
        Eigen::Vector3d q(qx, qy, qz - 7);

        // 生成速度指令
        Eigen::RowVector3d v = lambda * q;

        // 发布控制速度消息        
        control_vel.header = msg->header; // 使用速度消息的时间戳保持同步
        control_vel.twist.linear.x = v(0);
        control_vel.twist.linear.y = v(1);
        control_vel.twist.linear.z = v(2);
        control_vel.twist.angular.z = q_yaw;

        control_vel_pub_.publish(control_vel);
        ROS_INFO("Pub_vel_cmd: %f, %f, %f.", v(0), v(1), v(2));  
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber target_points_sub_;
    ros::Publisher control_vel_pub_;
    ros::Publisher image_pub_;

    std::vector<double> target_points_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "ibvs_controller");
    ros::NodeHandle nh;
    IBVSController controller(nh);
    ros::spin();
    return 0;
}