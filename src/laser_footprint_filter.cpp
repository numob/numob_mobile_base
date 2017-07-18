/*
 This file is ported from https://github.com/turtlebot/turtlebot_apps.git
 Package: turtlebot_navigation
 File: laser_footprint_filter.cpp
 Date: 2017/5/4
 */
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

class LaserFootprintFilter
{
public:
  LaserFootprintFilter()
    : nh_("~"), listener_(ros::Duration(10))
  {
    scan_filtered_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/scan", 1);
    scan_sub_ = nh_.subscribe("/scan_raw", 1000, &LaserFootprintFilter::update, this);

    nh_.param<double>("footprint_inscribed_radius", inscribed_radius_, 0.25*1.1);
    nh_.param<std::string>("base_frame", base_frame_, "/base_link");
  }

  void update(const sensor_msgs::LaserScan& input_scan)
  {
    sensor_msgs::LaserScan filtered_scan;
    filtered_scan = input_scan;

    double angle = filtered_scan.angle_min - filtered_scan.angle_increment;
    geometry_msgs::PointStamped p_input;
    p_input.header = input_scan.header;

    for(size_t i=0; i < filtered_scan.ranges.size(); i++)
    {
        angle += filtered_scan.angle_increment;
        if(filtered_scan.ranges[i] >= filtered_scan.range_max) continue;

        p_input.point.x = cos(angle) * filtered_scan.ranges[i];
        p_input.point.y = sin(angle) * filtered_scan.ranges[i];

        geometry_msgs::PointStamped p_transformed;        
        try{
            listener_.transformPoint(base_frame_, p_input, p_transformed);
        }catch(tf::TransformException &ex){
            ROS_ERROR("Received an exception trying to transform a point: %s", ex.what());
            return;
        }
    
        if( inFootprint(p_transformed) )
        {
            filtered_scan.ranges[i] = filtered_scan.range_max + 1.0;
        }

    }

    scan_filtered_pub_.publish(filtered_scan);
  }

  // Filter out circular area
  bool inFootprint(const geometry_msgs::PointStamped& scan_pt)
  {
    // Do a radius instead of a box.
    if (sqrt(scan_pt.point.x*scan_pt.point.x + scan_pt.point.y*scan_pt.point.y) > inscribed_radius_)
      return false;
    return true;
  }

private:
  ros::NodeHandle nh_;
  tf::TransformListener listener_;
  double inscribed_radius_;
  std::string base_frame_;
  ros::Publisher scan_filtered_pub_;
  ros::Publisher debug_pub_;
  ros::Subscriber scan_sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser_footprint_filter");

  LaserFootprintFilter filter;
  ros::spin();
}

