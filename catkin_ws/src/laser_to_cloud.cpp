#include <ros/ros.h>
#include <boost/bind/bind.hpp>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <math.h>
using namespace boost::placeholders;

laser_geometry::LaserProjection projector;

ros::Subscriber scan_sub;     // Subscriber for laser scan
ros::Publisher cloud_pub;     // Publisher for point cloud

// Callback function to process laser scan messages
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
  // Project the laser scan to a point cloud
  sensor_msgs::PointCloud2 cloud_msg;
  projector.projectLaser(*scan_in, cloud_msg);

  // Convert the point cloud message to a point cloud object
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(cloud_msg, *cloud);

	
  // RANSAC 객체 생성 (벽 검출을 위한 RANSAC)
  pcl::ModelCoefficients::Ptr wall_coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr wall_inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> wall_seg;

  wall_seg.setOptimizeCoefficients(true);
  wall_seg.setModelType(pcl::SACMODEL_PLANE); // 평면 모델 (벽)
  wall_seg.setMethodType(pcl::SAC_RANSAC);
  wall_seg.setMaxIterations(1000); // 반복 횟수
  wall_seg.setDistanceThreshold(0.01); // RANSAC에서 점이 모델에 속하는지 확인하는 임계값 (1cm 이내 포인트만 고려)

	// 만일 기존의 벽을 찾는다면 while 밖에 어떠한 변수 설정해놓고 
  // wall_normal_x와 wall_normal_y 비교해서 오차 주면 될 듯?


  ////////////////////////////////////
  ///            벽 찾기           ///
  ///////////////////////////////////
  // 벽을 찾기 위한 RANSAC 적용
  wall_seg.setInputCloud(cloud);

  std::vector<double> wall_angles;
  // 벽이 여러 개인 경우 처리
  while (cloud->points.size() > 0) {
    wall_seg.segment(*wall_inliers, *wall_coefficients);
    if (wall_inliers->indices.size() == 0) {
      break;  // 더 이상 벽이 없으면 종료
    }
    
    // PointIndices는 std::vector<int> indices로 이루어져 있음.
    // 벽의 점군을 추출하기
    // pcl::PointCloud<pcl::PointXYZ>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // wall_cloud->points.reserve(wall_inliers->indices.size());

    for (int index : wall_inliers->indices) {
      //wall_cloud->points.push_back(cloud->points[index]);
      ROS_INFO("wall index x is %f y is %f \n", cloud->points[index].x, cloud->points[index].y);
    }

    // 벽의 각도 계산 (예: 벽의 방향 벡터를 기준으로 각도 계산)
    // 벽의 방향 벡터(x,y,z)는 wall_coefficients->values[0], wall_coefficients->values[1], wall_coefficients->values[2]에 저장되어 있음
		double wall_normal_x = wall_coefficients->values[0];
		double wall_normal_y = wall_coefficients->values[1];
		double wall_normal_z = wall_coefficients->values[2];
		

		// imu 이용해서 방향 알 수 있고 unit vector로 구해야 함.
    // ros topic으로 수신?
		// double robot_direction_x = ...; 
		// double robot_direction_y = ...; 
		
		// 벽의 방향 벡터와 로봇의 방향 벡터 사이의 각도 계산 (라디안) 
		double wall_angle_radians = std::acos(wall_normal_x * robot_direction_x + wall_normal_y * robot_direction_y);
		
		// 라디안 값을 도(degree)로 변환
		double wall_angle_degrees = wall_angle_radians * 180.0 / M_PI;
		
    wall_angles.push_back(wall_angle_degrees);

    // 벽을 제거하고 다시 RANSAC을 적용하기 위해 해당 벽 점들을 클라우드에서 제거
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(wall_inliers);
    extract.setNegative(true);
    extract.filter(*cloud);
  }
  ////////////////////////////////////
  ///         장애물 찾기           ///
  ////////////////////////////////////  
  // RANSAC 객체 생성 (장애물 검출을 위한 RANSAC)
  pcl::ModelCoefficients::Ptr obstacle_coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr obstacle_inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> obstacle_seg;

  obstacle_seg.setOptimizeCoefficients(true);
  obstacle_seg.setModelType(pcl::SACMODEL_CYLINDER); // 원통 모델 또는 다른 모델로 설정 (장애물)
  obstacle_seg.setMethodType(pcl::SAC_RANSAC);
  obstacle_seg.setMaxIterations(1000); // 반복 횟수
  obstacle_seg.setDistanceThreshold(0.01); // RANSAC에서 점이 모델에 속하는지 확인하는 임계값


	int emergency = 0;
  // 장애물을 찾기 위한 RANSAC 적용
  obstacle_seg.setInputCloud(cloud);
  // 장애물이 여러 개인 경우 처리
	while (cloud->points.size() > 0) {
	  obstacle_seg.segment(*obstacle_inliers, *obstacle_coefficients);
	  if (obstacle_inliers->indices.size() == 0) {
	    break;  // 더 이상 장애물이 없으면 종료
	  }
	
	  // PointIndices는 std::vector<int> indices로 이루어져 있음.
	  // 장애물의 점군을 추출하기
	  // pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	  // obstacle_cloud->points.reserve(obstacle_inliers->indices.size());
	
	  for (int index : obstacle_inliers->indices) {
		   // obstacle_cloud->points.push_back(cloud->points[index]);
	    ROS_INFO("Obstacle index x is %f y is %f \n", cloud->points[index].x, cloud->points[index].y);
			// Emergency
			if(cloud->points[index].x < 0.2 || cloud->points[index].y < 0.2) {emergency = 1}
	  }
	
	  // 장애물을 제거하고 다시 RANSAC을 적용하기 위해 해당 장애물 점들을 클라우드에서 제거
	  pcl::ExtractIndices<pcl::PointXYZ> extract;
	  extract.setInputCloud(cloud);
	  extract.setIndices(obstacle_inliers);
	  extract.setNegative(true);
	  extract.filter(*cloud);
	}

  ////////////////////////////////////
  ///          주행 코드            ///
  ////////////////////////////////////  
  if (emergency == 0){
    
    double avg_x = 0.0;
    double avg_y = 0.0;
    int size = 0;
    for (const pcl::PointXYZ& point : cloud->points){
      // 정면에서만 회전 방향 설정
      if(point.x > 0){
        avg_x += point.x;
        avg_y += point.y;
        size++;
      }
    }
    avg_x /= size;
    avg_y /= size;

    double angle_radians = std::atan2(avg_y,avg_x);
    double angle_degree = angle_radians * 180.0 / M_PI;

    // 정면 장애물 Check 
    // y값으로 해야 할 듯?
    if ()

  }
  ////// Emergency stop //////
  else{

  }
  ////////////////////////////////////
			

  // Publish the point cloud object
  cloud_pub.publish(cloud);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_to_cloud");
  ros::NodeHandle nh;

  scan_sub = nh.subscribe("scan", 1, scanCallback);
  cloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("cloud", 1);

  ros::spin();

  return 0;
}