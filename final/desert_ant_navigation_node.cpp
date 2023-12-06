#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "desert_ant_navigation/pwm.hpp"

#define WHEEL_BASE 0.185  // meter
#define WHEEL_AXIS 0.180  // meter
#define CAR_SPEED 1.6    // m/s
#define MAX_STEER 0.4     // rad

#define LINEAR_ACCEL 0.5
#define LINEAR_DECEL 1
#define ANGULAR_ACCEL 1
#define ANGULAR_DECEL 1

PWM* THR_PWM = new PWM;
PWM* SER_PWM = new PWM;

float min_avoid_space_length;
double detect_distance;
double oa_threshold_;
double emergency_stop_threshold_;
int DDEBUG;
bool emergency_stop = false;
double tf_x=0, tf_y=0, tf_rot_x=0, tf_rot_y=0, od_x=0, od_y=0, od_rot_x=0, od_rot_y=0;

struct Point {
  double x;
  double y;
  Point() {}                                 // default constructor
  Point(double x, double y) : x(x), y(y) {}  // coustructor
};

struct Circle {
  Point center;
  double radius;
  Circle() {}                                                              // default constructor
  Circle(Point center, double radius) : center(center), radius(radius) {}  // coustructor
};

struct Pose {
  Point translation;  // {x, y}
  Point rotation;     // {cos(theta), sin(theta)}, vector size 1
  ros::Time timestamp;
  Pose() {}  // default constructor
  Pose(Point translation, Point rotation, ros::Time timestamp)
      : translation(translation), rotation(rotation), timestamp(timestamp) {}  // coustructor
};

struct CmdVel {
  double speed;  // m/s
  double steer;  // rad
  ros::Time timestamp;
  CmdVel() {}  // default constructor
  CmdVel(double speed, double steer, ros::Time timestamp) : speed(speed), steer(steer), timestamp(timestamp) {}
};

// double getDistanceTwoPointsByIdx(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int idx1, int idx2) {
//   Point p1 = {cloud->points[idx1].x, cloud->points[idx1].y};
//   Point p2 = {cloud->points[idx2].x, cloud->points[idx2].y};
//   return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
// }

double getDistanceTwoPoints(Point p1, Point p2) {
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// double getAngleThreePointsByIdx(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int idx) {
//   Point p1 = {cloud->points[idx - 2].x, cloud->points[idx - 2].y};
//   Point p2 = {cloud->points[idx - 1].x, cloud->points[idx - 1].y};
//   Point p3 = {cloud->points[idx].x, cloud->points[idx].y};
//   double a = getDistanceTwoPoints(p2, p3);
//   double b = getDistanceTwoPoints(p1, p3);
//   double c = getDistanceTwoPoints(p1, p2);
//   return 180 - std::acos((std::pow(a, 2) + std::pow(c, 2) - std::pow(b, 2)) / (2 * a * c)) * 180 / M_PI;
// }

double idx2angle(int idx, int size) { return idx * 2 * M_PI / size; }

Point RotateCoordinate(Point p, double theta, Point base) {
  // rotation matrix calculation
  Point ret;
  ret.x = (p.x - base.x) * cos(theta) - (p.y - base.y) * sin(theta) + base.x;
  ret.y = (p.x - base.x) * sin(theta) + (p.y - base.y) * cos(theta) + base.y;
  return ret;
}

Point RotateCoordinateOrigin(Point p, double theta) {
  // rotation matrix calculation
  Point ret;
  ret.x = p.x * cos(theta) - p.y * sin(theta);
  ret.y = p.x * sin(theta) + p.y * cos(theta);
  return ret;
}

tf::Quaternion getQuaternionFromRotationVector(double x, double y) {
  double yaw = std::atan2(y, x);
  tf::Quaternion q;
  q.setRPY(0, 0, yaw);
  return q;
}

class DesertAntNavigation {
 private:
  ros::NodeHandle nh_;
  ros::Subscriber scan_sub_;
  ros::Subscriber pose_sub_;
  ros::Subscriber odom_sub_;
  // ros::Publisher cloud_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher cmd_pub_;
  ros::Publisher pose_pub_;
  

  ros::Timer timer = nh_.createTimer(ros::Duration(0.02), &DesertAntNavigation::publishCmd, this);

  Pose pose_;               // current pose
  Pose als_pose_, odom_pose_, new_pose;           // Map pose
  Point goal_;              // global goal
  CmdVel dr_cmd_, oa_cmd_;  // dead reckoning, obstacle avoidance
  ackermann_msgs::AckermannDriveStamped prev_cmd_;
  bool obstacle_detected_ = false;
  int arrived = 0, arrived_back = 0;
  int stopping = 0, left_turn = 0;
  float stop_distance = 0.4;
  ros::Time init_time_ = ros::Time::now();
  ros::Time arrived_time_ = ros::Time::now();
  ros::Time pwm_time_ = ros::Time::now();

 public:
  DesertAntNavigation() {
    scan_sub_ = nh_.subscribe("/scan", 1, &DesertAntNavigation::obstacleAvoidance, this);
    pose_sub_ = nh_.subscribe("/mcl_pose", 1, &DesertAntNavigation::poseCallback, this);
    odom_sub_ = nh_.subscribe("/odom",1,&DesertAntNavigation::odomCallback, this);
    // cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/cloud", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/markers", 1);
    // cmd_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("/rc_car/ackermann_cmd", 1);
    // pub_sub_ = nh_.advertise<geometry_msgs::Pose>("/pose", 1);
    pose_ = {{0, 0}, {1/sqrt(2), 1/sqrt(2)}, ros::Time::now()};
    als_pose_ = {{0, 0}, {1, 0}, ros::Time::now()};
    odom_pose_ = {{0, 0}, {1, 0}, ros::Time::now()};

  }
  ~DesertAntNavigation() {}

  void setGoal(Point goal) { goal_ = goal; }

  void odomCallback(const nav_msgs::Odometry& msg){
    Pose new_pose;

    new_pose.translation.x = msg.pose.pose.position.x;
    new_pose.translation.y = msg.pose.pose.position.y;

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg.pose.pose.orientation,q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    new_pose.rotation.x = std::cos(yaw);
    new_pose.rotation.y = std::sin(yaw);
    visualizeOdom();
    // double cur_angle = obstacle_detected_ ? dr_cmd_.steer : oa_cmd_.steer;
    // if ((ros::Time::now().toSec() > init_time_.toSec() + 2) && (abs(cur_angle) > 0.1) && (getDistanceTwoPoints(new_pose.translation, odom_pose_.translation) < 0.3)) { 
    //   pose_ = odom_pose_;
    // }

    odom_pose_ = new_pose;

  }

  void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    Pose new_pose;
    // if (tf_x == 0 && tf_y == 0 && tf_rot_x == 0 && tf_rot_y == 0)
    // {
    //   tf_x = msg->pose.position.x+pose_.translation.x;
    //   tf_y = msg->pose.position.y+pose_.translation.y;
    //   tf::Quaternion q;
    //   tf::quaternionMsgToTF(msg->pose.orientation,q);
    //   tf::Matrix3x3 m(q);
    //   double roll, pitch, yaw;
    //   m.getRPY(roll, pitch, yaw);
    //   tf_rot_x = std::cos(yaw);
    //   tf_rot_y = std::sin(yaw);
    //   ROS_INFO("현재 위치 x: %lf y: %lf, rot_x : %lf rot_y: %lf \n", tf_x,tf_y,tf_rot_x,tf_rot_y);
    // }
    new_pose.translation.x = msg->pose.position.x;
    new_pose.translation.y = msg->pose.position.y;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation,q);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    new_pose.rotation.x = std::cos(yaw);
    new_pose.rotation.y = std::sin(yaw);
    double cur_angle = obstacle_detected_ ? dr_cmd_.steer : oa_cmd_.steer;
    if ((ros::Time::now().toSec() > init_time_.toSec() + 2) && (abs(cur_angle) > 0.1) && (getDistanceTwoPoints(new_pose.translation, als_pose_.translation) < 0.3)) { 
      pose_ = als_pose_;
    }

    als_pose_ = new_pose;

    visualizeAlsPose();
    std::cout << "pose received. x: " << new_pose.translation.x << " y: " << new_pose.translation.y << "rot x: " << new_pose.rotation.x << "rot y: " << new_pose.rotation.y <<std::endl;
  }

  void visualizePose() {
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(pose_.translation.x, pose_.translation.y, 0.0));
    tf::Quaternion q = getQuaternionFromRotationVector(pose_.rotation.x, pose_.rotation.y);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

    // publish marker which is an arrow
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "cur_pose";
    marker.id = marker.header.stamp.toNSec() / 100000000;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = pose_.translation.x;
    marker.pose.position.y = pose_.translation.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 0.25;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker_pub_.publish(marker);
  }

  void visualizeAlsPose() {
    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin(tf::Vector3(als_pose_.translation.x, als_pose_.translation.y, 0.0));
    tf::Quaternion q = getQuaternionFromRotationVector(als_pose_.rotation.x, als_pose_.rotation.y);
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

    // publish marker which is an arrow
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "als_pose";
    marker.id = marker.header.stamp.toNSec() / 100000000;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = als_pose_.translation.x;
    marker.pose.position.y = als_pose_.translation.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 0.25;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker_pub_.publish(marker);
  }

void visualizeOdom() {
    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin(tf::Vector3(odom_pose_.translation.x, odom_pose_.translation.y, 0.0));
    tf::Quaternion q = getQuaternionFromRotationVector(odom_pose_.rotation.x, odom_pose_.rotation.y);
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

    // publish marker which is an arrow
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "odom_pose";
    marker.id = marker.header.stamp.toNSec() / 100000000;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = odom_pose_.translation.x;
    marker.pose.position.y = odom_pose_.translation.y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = 0.25;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker_pub_.publish(marker);
  }

  double getAngleBetweenPoseAndGoal() {
    double theta = std::atan2(goal_.y - pose_.translation.y, goal_.x - pose_.translation.x);
    double theta_pose = std::atan2(pose_.rotation.y, pose_.rotation.x);
    return theta - theta_pose;
  }

  void deadReckoning(const ackermann_msgs::AckermannDriveStamped cmd_stamped) {
    CmdVel cmd_vel;
    cmd_vel.speed = cmd_stamped.drive.speed;
    cmd_vel.steer = cmd_stamped.drive.steering_angle;
    cmd_vel.timestamp = cmd_stamped.header.stamp;

    double dt = (cmd_vel.timestamp - pose_.timestamp).toSec();
    if (cmd_vel.speed) {
      if (cmd_vel.steer) {
        double R = (WHEEL_BASE + std::sqrt(std::pow(WHEEL_BASE, 2) + cmd_vel.steer * std::pow(WHEEL_AXIS, 2))) /
                   (2 * cmd_vel.steer) ;
        double omega = cmd_vel.speed / R;
        double dtheta = omega * dt;
        Point rotation_center = {pose_.translation.x - R * pose_.rotation.y,
                                 pose_.translation.y + R * pose_.rotation.x};
        Point new_translation = RotateCoordinate(pose_.translation, dtheta, rotation_center);
        Point new_rotation = RotateCoordinateOrigin(pose_.rotation, dtheta);
        pose_.translation = new_translation;
        pose_.rotation = new_rotation;
        // print R, omega, dtheta, rotation_center
        // std::cout << "R: " << R << ", omega: " << omega << ", dtheta: " << dtheta << ", rotation_center: ("
        //           << rotation_center.x << ", " << rotation_center.y << ")" << std::endl;
      } else {
        Point new_translation = {pose_.translation.x + cmd_vel.speed * pose_.rotation.x * dt,
                                 pose_.translation.y + cmd_vel.speed * pose_.rotation.y * dt};
        pose_.translation = new_translation;
      }
    }
    pose_.timestamp = cmd_vel.timestamp;

    visualizePose();

    // Update Cmd
    double steer = std::min(0.2, std::max(-0.2, getAngleBetweenPoseAndGoal()));
    double speed = dt > 0 && dt < 1 ? CAR_SPEED : 0;  // ignore if dt is too large
    dr_cmd_ = {speed, steer, ros::Time::now()};
  }

  void obstacleAvoidance(const sensor_msgs::LaserScan::ConstPtr& scan_in) {

    double min_obj_distance = 12.0;
    emergency_stop = false;
    double min_obj_angle = 0;

    std::vector<std::pair<double, double>> free_space_ranges;  // length, start
    std::vector<std::vector<double> > candidate_routes;        // candidate_routes : start_angle, end_angle, start_distance, end_distance, direct length
    // Scan (0 ~ M_PI 줄이기)
    double new_angle_min;
    double new_angle_max;

    int min_max_checker=0;
    int min_idx, max_idx;
    for (int i=0; i< scan_in->ranges.size();++i) {
      double curr_angle = scan_in->angle_min + scan_in->angle_increment * i;
      if((curr_angle) >= -1 * M_PI / 2 && (curr_angle) <= M_PI / 2) {
        if (scan_in->ranges[i] < min_obj_distance) {
          min_obj_distance = scan_in->ranges[i];
          min_obj_angle = curr_angle;
        }
        if (min_max_checker == 0) {
          new_angle_min = curr_angle;
          min_idx = i;
          min_max_checker = 1;
        }
        else if (min_max_checker == 1) {
          new_angle_max = curr_angle;
          max_idx = i;
        }
      }
    }

    double current_free_space_start = new_angle_min, current_free_space_start_distance = 12.0, current_free_space_end_distance = 12.0;
    bool in_free_space = true;

    for (size_t i = min_idx; i < max_idx; ++i) {
      if (scan_in->ranges[i] < detect_distance) {
        if (in_free_space) {
          double current_free_space_end = scan_in->angle_min + i * scan_in->angle_increment;
          current_free_space_end_distance = scan_in->ranges[i];
          double current_free_space_length = (current_free_space_end_distance+current_free_space_start_distance) * sin(std::abs(current_free_space_end - current_free_space_start)/2);
          if(current_free_space_length >= min_avoid_space_length*2) {
            std::vector<double> candidate_route{current_free_space_start, current_free_space_end, \
                current_free_space_start_distance, current_free_space_end_distance, current_free_space_length};
            candidate_routes.push_back(candidate_route);
          }
          in_free_space = false;
        }

      } 
      else if (!in_free_space) {
        current_free_space_start = scan_in->angle_min + i * scan_in->angle_increment;
        current_free_space_start_distance = scan_in->ranges[i-1];
        in_free_space = true;
      }
      if (i == max_idx - 1) {
        if (in_free_space) {
          double current_free_space_end = new_angle_max;
          double current_free_space_end_distance = 12.0;
          double current_free_space_length = (current_free_space_end_distance+current_free_space_start_distance) * sin(std::abs(current_free_space_end - current_free_space_start)/2);
          if(current_free_space_length >= min_avoid_space_length*2) {
            std::vector<double> candidate_route{current_free_space_start, current_free_space_end, \
                current_free_space_start_distance, current_free_space_end_distance, current_free_space_length};
            candidate_routes.push_back(candidate_route);
          }
        }
      }
    }
    obstacle_detected_ = min_obj_distance < oa_threshold_ ? true : false;

    float opt_avoid_angle = -1*M_PI, opt_avoid_angle_diff = M_PI, obj_angle = dr_cmd_.steer;
    // candidate_routes : start_angle, end_angle, start_distance, end_distance, direct length
    for (auto candidate_route : candidate_routes) {
      if (candidate_route[0] < obj_angle && candidate_route[1] > obj_angle) {
        double possible_angle_min = asin(min_avoid_space_length / candidate_route[2]) + candidate_route[0];
        double possible_angle_max = candidate_route[1] - asin(min_avoid_space_length / candidate_route[3]);
        if(possible_angle_min <= obj_angle && possible_angle_max >= obj_angle){
          opt_avoid_angle = obj_angle;
        }
        else {
          opt_avoid_angle = possible_angle_min > obj_angle ? possible_angle_min : possible_angle_max;
        }
        opt_avoid_angle_diff = std::abs(opt_avoid_angle - obj_angle);
        break;
      }
      else {
        double tmp_opt_avoid_angle = candidate_route[0] > obj_angle ? asin(min_avoid_space_length / candidate_route[2]) + candidate_route[0] : candidate_route[1] - asin(min_avoid_space_length / candidate_route[3]);
        double tmp_opt_avoid_angle_diff = std::abs(opt_avoid_angle - obj_angle);
        if (tmp_opt_avoid_angle_diff < opt_avoid_angle_diff) {
          opt_avoid_angle = tmp_opt_avoid_angle;
          opt_avoid_angle_diff = tmp_opt_avoid_angle_diff;
        }
      }
    }
    emergency_stop = (candidate_routes.size() == 0 || min_obj_distance < emergency_stop_threshold_) ? true : false;
    double steer = std::min((float)0.3, std::max((float)-0.3, opt_avoid_angle));
    double speed = CAR_SPEED-0.5;
    if (emergency_stop) {
      speed = 0;
      // steer = min_obj_angle > 0 ? -0.4 : 0.4;
      steer = 0;
    }
    if(!arrived) left_turn = (scan_in->ranges[min_idx] < scan_in->ranges[max_idx]) ? 1 : -1;
    oa_cmd_ = {speed, steer, ros::Time::now()};
    std::cout << "///////////////Left Turn ////////////// : " << left_turn <<std::endl;
    // print dr_cmd_ and time
    // std::cout << "////////////////경로 후보///////////////////////" << std::endl;
    // std::cout << "시작 각\t\t끝 각\t\t시작 거리  끝 거리  열린 공간의 길이\n";
    // for (int i=0; i< candidate_routes.size(); i++){
    //   for (int j=0; j< candidate_routes[i].size(); j++) {
    //     if (j<2) std::cout << candidate_routes[i][j] *180/M_PI << "도\t";
    //     else std::cout << candidate_routes[i][j] << "\t";
    //   }
    //   std::cout << std::endl;
    // }
    // std::cout << " /////////////////////////////////////////////////// " << std::endl;
  }

  void publishCmd(const ros::TimerEvent& event) {
    // std::cout << "dr_cmd_: (" << dr_cmd_.speed << ", " << dr_cmd_.steer << "), " << dr_cmd_.timestamp << std::endl;
    // std::cout << "oa_cmd_: (" << oa_cmd_.speed << ", " << oa_cmd_.steer << "), " << oa_cmd_.timestamp << std::endl;
    int returning = 0;
    ackermann_msgs::AckermannDriveStamped des_vel_stamped;
    if (obstacle_detected_) {
      des_vel_stamped.drive.speed = oa_cmd_.speed;
      des_vel_stamped.drive.steering_angle = oa_cmd_.steer;
      des_vel_stamped.header.stamp = oa_cmd_.timestamp;
    } else {
      des_vel_stamped.drive.speed = dr_cmd_.speed;
      des_vel_stamped.drive.steering_angle = dr_cmd_.steer;
      des_vel_stamped.header.stamp = dr_cmd_.timestamp;
    }

    if (ros::Time::now().toSec() - init_time_.toSec() < 2) {
      des_vel_stamped.drive.steering_angle = 0;
    }

    // speed = 0 when arrived to goal
    // 현재 rotation 
    // 
    if (getDistanceTwoPoints(pose_.translation, goal_) < stop_distance) {
      arrived = 1;
      // returning = 1;
      arrived_time_ = ros::Time::now();
      stop_distance=1.0;
      if (goal_.x == 0 && goal_.y ==0)
        arrived_back = 1;
      goal_.x = 0;
      goal_.y = 0;
      pose_.translation.x += 1.0 * pose_.rotation.x;  // 1.0m: 관성으로 가는 거리
      pose_.translation.y += 1.0 * pose_.rotation.y;  // 1.0m: 관성으로 가는 거리
    }

    // 중간 경유지점 만들어서 돌아가기 편하게 설정
    // if (arrived == 1 && goal_.x != 0 && goal_.y != 0)
    // {
    //   double rotation_radius = 1.2;
    //   // __________________________ 시험적인 코드!!!! __________________________
    //   // 파악하고 도달하고 바꿔야 하지 않나? 계속 업데이트 를 하니까 뱅글뱅글 도는 것 같기도? -> arrived=0으로 초기화 
    //   // 다시 원점과 현재 방향 사이의 각도 파악
    //   double origin_x = pose_.translation.x/std::sqrt(pose_.translation.x*pose_.translation.x + pose_.translation.y*pose_.translation.y);
    //   double origin_y = pose_.translation.y/std::sqrt(pose_.translation.x*pose_.translation.x + pose_.translation.y*pose_.translation.y);
    //   if((origin_x*pose_.rotation.x + origin_y*pose_.rotation.y) < cos(30 / 180 * M_PI)) // 내적해서 30도 이상
    //   {
    //     // origin이 아니라 pose_.rotation 기준으로 해야 하지 않나??
    //     goal_.x = pose_.translation.x + pose_.rotation.x + rotation_radius * (cos(M_PI/6)*pose_.rotation.x + left_turn*sin(M_PI/6)*pose_.rotation.y);
    //     goal_.y = pose_.translation.y + pose_.rotation.y + rotation_radius * (cos(M_PI/6)*pose_.rotation.y - left_turn*sin(M_PI/6)*pose_.rotation.x);
    //     arrived = 0;
      
    //   }
    //   else {
    //     returning = 0;
    //     goal_.x = 0;
    //     goal_.y = 0;
    //   }
    // }

    double dt = des_vel_stamped.header.stamp.toSec() - prev_cmd_.header.stamp.toSec();
    if (dt > 1e-18) {
      // print des_vel_stamped and prev_cmd_
      double dvx = des_vel_stamped.drive.speed - prev_cmd_.drive.speed;
      double dvz = des_vel_stamped.drive.steering_angle - prev_cmd_.drive.steering_angle;

      // only when the difference is greater than the acceleration, we will accelerate
      if (dvx > LINEAR_ACCEL * dt)
        des_vel_stamped.drive.speed = prev_cmd_.drive.speed + LINEAR_ACCEL * dt;
      else if (dvx < -1 * LINEAR_DECEL * dt)
        des_vel_stamped.drive.speed = prev_cmd_.drive.speed - LINEAR_DECEL * dt;

      if (dvz > ANGULAR_ACCEL * dt)
        des_vel_stamped.drive.steering_angle = prev_cmd_.drive.steering_angle + ANGULAR_ACCEL * dt;
      else if (dvz < -1 * ANGULAR_DECEL * dt)
        des_vel_stamped.drive.steering_angle = prev_cmd_.drive.steering_angle - ANGULAR_DECEL * dt;

    }
    if (des_vel_stamped.drive.speed == 0) {
      des_vel_stamped.drive.steering_angle == 0;
    }

    // arrived 제거
    if ((arrived && ((ros::Time::now().toSec() - arrived_time_.toSec()) < 3)) || arrived_back == 1) {
      des_vel_stamped.drive.speed = 0;
      des_vel_stamped.drive.steering_angle = 0;
    }
    if ((arrived && ((ros::Time::now().toSec() - arrived_time_.toSec()) < 5)) || arrived_back == 1) {
      des_vel_stamped.drive.steering_angle = 0;
    }
    // if (arrived_back == 1) {
    //   des_vel_stamped.drive.speed = 0;
    //   des_vel_stamped.drive.steering_angle = 0;
    // }

    // if (returning == 1) {
    //   des_vel_stamped.drive.speed = CAR_SPEED - 0.8;
    // }


    prev_cmd_.drive.speed = des_vel_stamped.drive.speed;
    prev_cmd_.drive.steering_angle = des_vel_stamped.drive.steering_angle;
    prev_cmd_.header.stamp = des_vel_stamped.header.stamp;


    // PWM 제작
    double angle = des_vel_stamped.drive.steering_angle;
    // SER(0.056(Right) ~ 0.072(Stay) ~ 0.088(Left))
    // 좌회전 (0 ~ 0.4 -> 0.072 ~ 0.088)
    // 우회전 (-0.4 ~ 0 -> 0.056 ~ 0.072)
    angle = angle * (0.088-0.072) / (0 + 0.4) + 0.0712; 

    double thr_pwm;
    if (des_vel_stamped.drive.speed == 0){
      thr_pwm = 0.0720;
    } else if (des_vel_stamped.drive.speed < 0.9) {
      thr_pwm = des_vel_stamped.drive.speed / 0.9 * 0.0027 + 0.0750 - 0.0006;
    } else if (des_vel_stamped.drive.speed < 1.3) {
      thr_pwm = (des_vel_stamped.drive.speed - 0.9) / 0.4 * 0.0006 + 0.0777 - 0.001;
    } else if (des_vel_stamped.drive.speed < 2.0) {
      thr_pwm = (des_vel_stamped.drive.speed - 1.3) / 0.7 * 0.0003 + 0.0783 -0.001;
    } else {
      thr_pwm = 0.0720;
    }


    // if (ros::Time::now().toSec() - pwm_time_.toSec() > 0.5)
    // {
    //   thr_pwm = 0.72;
    // }

    
    if (ros::Time::now().toSec() - init_time_.toSec() < 2) {
      thr_pwm = (thr_pwm - 0.0750) * 1.1 + 0.0750;
    }
    if (ros::Time::now().toSec() - arrived_time_.toSec() < 3){
      des_vel_stamped.drive.speed=0;
      thr_pwm = 0.0720;
    }
    // else
    // {
    //   pwm_time_ = ros::Time::now();
    // }



    // if (arrived && stopping == 0){
    //   thr_pwm = 0.05;
    //   stopping = 1;
    // }

    THR_PWM->testing(thr_pwm);
    SER_PWM->testing((angle - 0.072) *1.0 + 0.072);
    deadReckoning(des_vel_stamped);
    // std::cout << "m/s speed: " << des_vel_stamped.drive.speed << " angle: " << des_vel_stamped.drive.steering_angle << " dt: " << dt << std::endl;
    std::cout << "AAArived : " << arrived << " goal_x : " << goal_.x << " goal_y : " << goal_.y << "  rot_x :  " <<  pose_.rotation.x << "  rot_y  " << pose_.rotation.y << std::endl;
    std::cout << "time: " << ros::Time::now() << " pose: (" << pose_.translation.x << ", " << pose_.translation.y << "), "
              << std::atan2(pose_.rotation.y, pose_.rotation.x) << "rad" << " state: " << obstacle_detected_ << " emergency: " << emergency_stop
              << " speed m/s: " << des_vel_stamped.drive.speed << " steer: " << des_vel_stamped.drive.steering_angle
              << " thr_pwm: " << thr_pwm << " ser_pwm: " << angle << std::endl;
  }
};

int main(int argc, char** argv) {
  // wiringpi
  wiringPiSetupGpio();
  
  THR_PWM->setpin(13);
  SER_PWM->setpin(18);

  for (int i = 0; i < 20; i++) {
    THR_PWM->testing(0.072);
    SER_PWM->testing(0.072);
  }

  // yaml 파일에서 읽기
  // std::ifstream ifs("config1.yaml");
  YAML::Node config = YAML::LoadFile("/home/ubuntu/desert_ant_navigation_ws/src/desert_ant_navigation/src/config1.yaml");
  min_avoid_space_length = config["min_avoid_space_length"].as<float>();
  detect_distance = config["detect_distance"].as<double>();
  oa_threshold_ = config["oa_threshold_"].as<double>();
  emergency_stop_threshold_ = config["emergency_stop_threshold_"].as<double>();
  DDEBUG = config["DDEBUG"].as<int>();
  // ifs.close();

  ros::init(argc, argv, "desert_ant_navigation_node");

  DesertAntNavigation desert_ant_navigation;  
  // get goal value from command line
  double goal_x = std::atof(argv[1]);
  double goal_y = std::atof(argv[2]);
  Point goal = {goal_x, goal_y};
  desert_ant_navigation.setGoal(goal);
  
  ros::spin();

  return 0;
}