#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <cmath>

// 转换函数 从tf::Quaternion 到 geometry_msgs::Quaternion
geometry_msgs::Quaternion tfQuatToMsg(const tf::Quaternion &tf_quat) {
    geometry_msgs::Quaternion msg_quat;
    msg_quat.x = tf_quat.x();
    msg_quat.y = tf_quat.y();
    msg_quat.z = tf_quat.z();
    msg_quat.w = tf_quat.w();
    return msg_quat;
}

int main(int argc, char **argv) {
  // 初始化ROS节点
  ros::init(argc, argv, "block_controller");
  ros::NodeHandle nh;

  // 创建服务客户端
  ros::ServiceClient set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  ros::Publisher Target_Vel_pub = nh.advertise<geometry_msgs::TwistStamped>("ground_target_Vel", 50);
  ros::Publisher Target_Pos_pub = nh.advertise<geometry_msgs::PoseStamped>("ground_target_Pos", 50);

  // 设置更新频率
  ros::Rate rate(50);  // 50 Hz
  double t = 0.0;  // 初始时间
  //const double omega = 0.5;  // 角速度
  const double omega = 0.25;  // 角速度
  const double radius = 2.0;  // 半径
  const double x_center = 0.0, y_center = 0.0;  // 圆心坐标

  while (ros::ok()) {
    // 计算位置
    double x = radius * cos(omega * t) + x_center;
    double y = radius * sin(omega * t) + y_center;

    // 计算速度矢量
    double vx = -radius * omega * sin(omega * t);
    double vy = radius * omega * cos(omega * t);

    // 计算朝向
    tf::Quaternion q;
    q.setRPY(0, 0, atan2(vy, vx));

    // 创建Pose和Twist对象
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation = tfQuatToMsg(q);

    geometry_msgs::Twist twist;
    twist.linear.x = vx;
    twist.linear.y = vy;

    // 创建ModelState对象
    gazebo_msgs::ModelState model_state;
    model_state.model_name = "flat_plate";  // 替换为你的方块模型名称
    model_state.pose = pose;
    model_state.twist = twist;

    // 请求设置模型状态
    gazebo_msgs::SetModelState srv;
    srv.request.model_state = model_state;
    if (set_model_state_client.call(srv)) {
      // 如果调用成功，则继续
    } else {
      ROS_ERROR("Failed to call service /gazebo/set_model_state");
      //return 1;
    }

    // 创建带时间戳位置和速度
    geometry_msgs::PoseStamped target_pose;
    target_pose.pose.position.x = x;
    target_pose.pose.position.y = y;
    target_pose.pose.orientation = tfQuatToMsg(q);
    target_pose.header.stamp = ros::Time::now();
    Target_Pos_pub.publish(target_pose);
    
    geometry_msgs::TwistStamped target_twist;
    target_twist.twist.linear.x = vx;
    target_twist.twist.linear.y = vy;
    target_twist.twist.linear.z = 0;
    target_twist.header.stamp = ros::Time::now();
    Target_Vel_pub.publish(target_twist);


    // 更新时间
    t += 0.02;  // 假设循环周期为0.02秒
    rate.sleep();
  }

  return 0;
}