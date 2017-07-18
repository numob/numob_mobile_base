#ifndef NUMOB_ROBOT_ROS_NODE_H
#define NUMOB_ROBOT_ROS_NODE_H

#include <numob/sdk.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <tf/tfMessage.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>


//params
double ros_rate = 20.0;
double wheel_diameter = 0.0; //will get value from mobile base
double wheel_track = 0.0; //will get value from mobile base
int wheel_encoder_resolution = 672; //wheel encoder ticks per revolution.
std::vector<double> ultrasound_offsets = {0.25, 0.25, 0.25, 0.25, 0.25}; //meter. The distance of ultrasound sensor from the center of the mobile base.
std::vector<double> ultrasound_angles = {31.64, 0.0, -31.64, -146.76, -213.24}; //the installation angles of each sensor.
std::vector<int>    ultrasound_used = {0,1,2,3,4}; // which sensor will be used. 0 based sensor location, its index matches the index of vector _ultrasound_angles
double ultrasound_max_valid_range = 0.6; //meter. Value beyond this value will be set to a big enough value in point cloud.
int paramtest = 100;
//internal
double _degree_to_radian_factor = 3.14159/180.0;
double _imu_timestamp = 0.0;

void velocity_callback(const geometry_msgs::Twist& command);

//odom and tf broadcast
void publish_odom(ros::Publisher publisher_odom, tf::TransformBroadcaster tf_odom_baselink_broadcaster, ros::Time current_time, double x, double y, double th, double vxy, double vth);

//pointCloud and tf broadcast
void publish_ultrasound(ros::Publisher publisher_ultrasound, tf::TransformBroadcaster tf_baselink_ultrasound_broadcaster);

//IMU and tf broadcast
void publish_imu(ros::Publisher publisher_imu, tf::TransformBroadcaster tf_baselink_imulink_broadcaster);


#endif
