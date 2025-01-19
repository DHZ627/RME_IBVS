#include <ros/ros.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>

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
  ros::init(argc, argv, "square_path_controller");
  ros::NodeHandle nh;

  // 创建服务客户端
  ros::ServiceClient set_model_state_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  ros::Publisher Target_Vel_pub = nh.advertise<geometry_msgs::TwistStamped>("ground_target_Vel", 50);
  ros::Publisher Target_Pos_pub = nh.advertise<geometry_msgs::PoseStamped>("ground_target_Pos", 50);


  // 设置更新频率
  ros::Rate rate(50);  // 50 Hz

  // 定义正方形边长和目标速度
  const double side_length = 1.0;  // 正方形边长
  const double target_speed = 0.2;  // 目标速度 (m/s)
  const int num_segments = 4;  // 正方形有四条边
  const double yaw_angle = 0.0;  // 固定偏航角度

  // 初始化起始位置和方向
  double x = -side_length / 2.0 + 0.01;  // 略微偏离起点以避免数值问题
  double y = -side_length / 2.0 + 0.01;
  tf::Quaternion q;
  q.setRPY(0, 0, yaw_angle);

  int segment = 0;  // 当前段索引 (0-3)
  double t = 0.0;   // 时间变量

  while (ros::ok()) {
    double vx = 0.0, vy = 0.0;

    switch (segment % num_segments) {
      case 0: // Move right
        x = -side_length / 2.0 + target_speed * t;
        vy = 0.0;
        vx = target_speed;
        break;
      case 1: // Move up
        y = -side_length / 2.0 + target_speed * t;
        vx = 0.0;
        vy = target_speed;
        break;
      case 2: // Move left
        x = side_length / 2.0 - target_speed * t;
        vy = 0.0;
        vx = -target_speed;
        break;
      case 3: // Move down
        y = side_length / 2.0 - target_speed * t;
        vx = 0.0;
        vy = -target_speed;
        break;
    }

    // 检查是否需要转换到下一个段
    if (t >= (side_length / target_speed)) {
      t = 0.0;
      ++segment;
      segment %= num_segments;

      // 更新起始位置为下一段的起点
      switch (segment) {
        case 0:
          x = -side_length / 2.0 + 0.01;  // 略微偏离起点以避免数值问题
          y = -side_length / 2.0 + 0.01;
          break;
        case 1:
          x = side_length / 2.0 - 0.01;  // 略微偏离终点以避免数值问题
          y = -side_length / 2.0 + 0.01;
          break;
        case 2:
          x = side_length / 2.0 - 0.01;
          y = side_length / 2.0 - 0.01;
          break;
        case 3:
          x = -side_length / 2.0 + 0.01;
          y = side_length / 2.0 - 0.01;
          break;
      }
    }

    // 创建Pose和Twist对象并发布
    geometry_msgs::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.orientation = tfQuatToMsg(q);

    geometry_msgs::Twist twist;
    twist.linear.x = vx;
    twist.linear.y = vy;

    // 创建ModelState对象并请求设置模型状态
    gazebo_msgs::ModelState model_state;
    model_state.model_name = "flat_plate";  // 替换为您的模型名称
    model_state.pose = pose;
    model_state.twist = twist;

    gazebo_msgs::SetModelState srv;
    srv.request.model_state = model_state;
    if (!set_model_state_client.call(srv)) {
      ROS_ERROR("Failed to call service /gazebo/set_model_state");
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
    t += 0.02;
    rate.sleep();
  }

  return 0;
}