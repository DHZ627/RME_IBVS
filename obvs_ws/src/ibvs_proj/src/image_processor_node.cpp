//在ROS中从相机中获取时间戳同步的图像和相机信息以及无人机IMU数据并在同一个回调函数中进行相机图片进行投影变换，最终将结果返回出来
#include <ros/ros.h>
#include <iostream>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Float64MultiArray.h>
#include "ibvs_proj/img_moment.h"


static const std::string OPENCV_WINDOW = "Image window";
double yaw_;

class CameraIMUSync {
public:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Publisher image_pub_;
  ros::Publisher points_pub_;

  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub_;
  message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Imu> MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync_;

  

  CameraIMUSync() : it_(nh_), sync_(MySyncPolicy(10), image_sub_, cam_info_sub_, imu_sub_) 
  {
    // 订阅相机图像、相机信息和IMU数据
    image_sub_.subscribe(nh_, "iris/usb_cam/image_raw", 30);
    cam_info_sub_.subscribe(nh_, "iris/usb_cam/camera_info", 30);
    imu_sub_.subscribe(nh_, "mavros/imu/data", 50);

    // 注册同步回调函数
    sync_.registerCallback(boost::bind(&CameraIMUSync::syncCallback, this, _1, _2, _3));

    // 发布处理后的图像
    image_pub_ = it_.advertise("IBVS/processed_image", 30);

    // 发布角点数据话题
    points_pub_ = nh_.advertise<ibvs_proj::img_moment>("target_points", 30);

    // Create a window for display.
    cv::namedWindow(OPENCV_WINDOW);

  }

    ~CameraIMUSync()
    {
      cv::destroyWindow(OPENCV_WINDOW);
    }

  // 生成虚拟相平面
  cv::Mat virtual_img_plane(cv_bridge::CvImagePtr cv_ptr, const sensor_msgs::CameraInfoConstPtr &cam_info_msg, const sensor_msgs::ImuConstPtr &imu_msg)
  {
    // 获取相机内参
    cv::Mat K(3, 3, CV_32F);
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        K.at<float>(i, j) = cam_info_msg->K[i * 3 + j];
      }
    }
    // std::cout << "Matrix K:\n" << K << std::endl;

    // 进行投影变换

    // 获取IMU数据
    const auto& orientation = imu_msg->orientation;
    // 将FLU系中的四元数转换为FRD系中的四元数,y,z取负【mavros默认为FLU系】
    tf::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
    tf::Matrix3x3 Rotation(q);

    // 四元数解算姿态角
    double r, p, y, roll, pitch, yaw;
    Rotation.getRPY(r, p, y);
    yaw_ = y;

    roll =  -p;
    pitch = -r;
    yaw = - y;
    // std::cout << "roll:\n" << roll << std::endl;
    // std::cout << "pitch:\n" << pitch << std::endl;
    //构建旋转矩阵
    cv::Mat Rx = (cv::Mat_<float>(3, 3) << 
        1, 0, 0,
        0, cos(roll), -sin(roll),
        0, sin(roll), cos(roll));

    cv::Mat Ry = (cv::Mat_<float>(3, 3) << 
        cos(pitch), 0, sin(pitch),
        0, 1, 0,
        -sin(pitch), 0, cos(pitch));

    // 获取不包含偏航角的旋转矩阵
    cv::Mat Rz = (cv::Mat_<float>(3, 3) << 
        cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1);

    cv::Mat E = (cv::Mat_<float>(3, 3) << 
        1, 0, 0,
        0, 1, 0,
        0, 0, 1);


    // cv::Mat R = E * Ry * Rx;
    cv::Mat R = Rx * Ry;
    // cv::Mat R = Rz;


    // cv::Mat R;
    // cv::multiply(Ry, Rx, R);
        
    //std::cout << "Matrix R:\n" << R << std::endl;

    // 计算矩阵H = K * R * K_inv
    cv::Mat H = K * R * K.inv();

    //矩阵输出
    // std::cout << "Matrix H:\n" << H << std::endl;

    cv::Mat warped_image;
    cv::warpPerspective(cv_ptr->image, warped_image, H, cv_ptr->image.size(), CV_INTER_CUBIC);

    return warped_image;
  }
    
  //图像处理，生成图像矩
  void processImage(cv::Mat &frame) 
  {

      // 创建消息
      ibvs_proj::img_moment target_msg;

      // 转换为灰度图像
      cv::Mat gray;
      cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

      // 二值化图像
      cv::Mat binary;
      cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);

      // 查找轮廓
      std::vector<std::vector<cv::Point>> contours;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(binary, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

      // 遍历轮廓
      for (size_t i = 0; i < contours.size(); ++i) {
          // 近似多边形
          std::vector<cv::Point> approx;
          cv::approxPolyDP(contours[i], approx, cv::arcLength(contours[i], true) * 0.02, true);

          // 检查是否为四边形
          if (approx.size() == 4) {
              // 计算面积
              double area = cv::contourArea(approx);
              if (area > 1000) { // 最小面积阈值
                  // 绘制轮廓
                  cv::drawContours(frame, std::vector<std::vector<cv::Point>>(1, approx), -1, cv::Scalar(0, 0, 255), 5);
                  int i = 0;
                  // 打印角点
                  for (const auto &point : approx) {
                    ROS_INFO("Corner: (%d, %d)", point.x, point.y);
                    //std_msg下还包含了基本数据类型的向量 向量的赋值要格外注意，应该使用pushback()函数来填充数据
                    target_msg.data.push_back(point.x);
                    target_msg.data.push_back(point.y); 
                }
                ROS_INFO("Find Target!");
                target_msg.data.push_back(yaw_);
                target_msg.header.stamp = ros::Time::now();
                //发布目标点数据
                points_pub_.publish(target_msg);
            }
        }
      }
  }


  void syncCallback(const sensor_msgs::ImageConstPtr& img_msg,
                    const sensor_msgs::CameraInfoConstPtr& cam_info_msg,
                    const sensor_msgs::ImuConstPtr& imu_msg) 
  {
    // 将图像消息转换为OpenCV格式
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // 生成虚拟相平面
    cv::Mat virtual_img;
    virtual_img = virtual_img_plane(cv_ptr, cam_info_msg, imu_msg);

    //图像处理，生成图像矩阵
    processImage(virtual_img);

    // 图片大小重新设置
    // 计算新的宽度和高度
    int new_width = cv_ptr->image.cols / 4;
    int new_height = cv_ptr->image.rows / 4;

    // 调整图像大小
    cv::Mat resized_image_01;cv::Mat resized_image_02;
    cv::resize(cv_ptr->image, resized_image_01, cv::Size(new_width, new_height));
    cv::resize(virtual_img, resized_image_02, cv::Size(new_width, new_height));
    // 合并两个图像
    cv::Mat combined_image;
    cv::vconcat(resized_image_01, resized_image_02, combined_image);

    // 显示图片
    cv::imshow(OPENCV_WINDOW, combined_image);
    cv::waitKey(1);

    // 发布处理后的图像
    sensor_msgs::ImagePtr processed_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", virtual_img).toImageMsg();
    image_pub_.publish(processed_img_msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_proc_node");
  CameraIMUSync cis;

  ros::spin();
  return 0;
}