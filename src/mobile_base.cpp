#include "numob/mobile_base.h"

numob::MobileBase mobileBase;


int main(int argc, char **argv) {
    ros::init(argc, argv, "numob_robot");

    ros::NodeHandle n;
    ros::NodeHandle private_n("~");

    private_n.param("ros_rate", ros_rate, ros_rate);
    ros::Rate loop_rate(ros_rate);

    ros::Publisher publisher_odom = n.advertise<nav_msgs::Odometry>("odom", 20);
    tf::TransformBroadcaster tf_odom_baselink_broadcaster;

    ros::Publisher publisher_ultrasound = n.advertise<sensor_msgs::PointCloud>("ultrasound", 20);
    tf::TransformBroadcaster tf_baselink_ultrasound_broadcaster;

    ros::Publisher publisher_imu = n.advertise<sensor_msgs::Imu>("imu", 20);
    tf::TransformBroadcaster tf_baselink_imulink_broadcaster;

    ros::Subscriber sub2 = n.subscribe("/cmd_vel", 1000, velocity_callback);
    //remoter_vel is from the remoter, such as iphone app
    ros::Subscriber sub3 = n.subscribe("/remoter_vel", 1000, velocity_callback);

    // params
    private_n.param("wheel_diameter", wheel_diameter, wheel_diameter);
    private_n.param("wheel_track", wheel_track, wheel_track);
    private_n.param("wheel_encoder_resolution", wheel_encoder_resolution, wheel_encoder_resolution);
    private_n.param("ultrasound_offsets", ultrasound_offsets, ultrasound_offsets);
    private_n.param("ultrasound_used", ultrasound_used, ultrasound_used);
    private_n.param("ultrasound_angles", ultrasound_angles, ultrasound_angles);
    private_n.param("ultrasound_max_valid_range", ultrasound_max_valid_range, ultrasound_max_valid_range);



    //odom
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    double vxy = 0.0;
    double vth = 0.0;

    double last_left_distance = 0.0; //last read distance from wheel
    double last_right_distance = 0.0; // last read distance from wheel

    ros::Time current_time = ros::Time::now();
    ros::Time last_time_odom = current_time;
    ros::Time last_time_health_check = current_time;

    //connect to mobile base  until success
    bool connect_result = false;
    while (!connect_result && ros::ok()) {
        connect_result = mobileBase.connect("/dev/numob", true);
        if(!connect_result)ros::Duration(0.5).sleep();

    }

    //set mobile base configuration, until success.
    bool set_param_result = false;
    while(!set_param_result && ros::ok()) {
        set_param_result = mobileBase.setWheelDiameter(wheel_diameter);
        if(!set_param_result)ros::Duration(0.5).sleep();
    }
    set_param_result = false;
    while(!set_param_result && ros::ok()) {
        set_param_result = mobileBase.setWheelTrack(wheel_track);
        if(!set_param_result)ros::Duration(0.5).sleep();
    }
    set_param_result = false;
    while(!set_param_result && ros::ok()) {
        set_param_result = mobileBase.setWheelEncoderResolution(wheel_encoder_resolution);
        if(!set_param_result)ros::Duration(0.5).sleep();
    }


    //reset wheels distance until success
    bool result_reset_wheel_travelled_distance = false;
    while (!result_reset_wheel_travelled_distance && ros::ok()) {
        result_reset_wheel_travelled_distance = mobileBase.resetWheelTravelledDistance({1,2});
        if(!result_reset_wheel_travelled_distance)ros::Duration(0.5).sleep();
    }

    //reset imu heading until success
    bool result_reset_heading = false;
    while (!result_reset_heading  && ros::ok()) {
        result_reset_heading = mobileBase.resetHeadingOffset();
        if(!result_reset_heading)ros::Duration(0.5).sleep();
    }



    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep(); //sleep at top.

        current_time = ros::Time::now();


        //odom
        std::vector<double> v_odom_data = mobileBase.getOdometry();
        if (v_odom_data.empty()) {
            continue; //skip this loop if odom data read fail.
        }
        //validate the imu update timestamp
        if(_imu_timestamp!=0.0 && v_odom_data.at(4) > (_imu_timestamp + 1000)){
            ROS_ERROR_STREAM( "IMU is not updated in 1000ms.\n");
        }
        _imu_timestamp = v_odom_data.at(4);

        //compute odometry in a typical way given the velocities of the robot
        double dt = (current_time - last_time_odom).toSec();

        double delta_left = v_odom_data.at(0) - last_left_distance;
        double delta_right = v_odom_data.at(1) - last_right_distance;
        double dxy_ave = (delta_left + delta_right) / 2.0;

        double delta_th = 0.0;
        double imu_heading = 0.0;
        long imu_timestamp = 0;

        imu_heading = v_odom_data.at(2);
        vth = v_odom_data.at(3);
        delta_th = imu_heading - th;

        vxy = dxy_ave / dt;

        double delta_x = 0.0;
        double delta_y = 0.0;
        if (dxy_ave != 0.0) {
            double dx = cos(delta_th) * dxy_ave;
            double dy = -sin(delta_th) * dxy_ave;
            delta_x = (cos(th) * dx - sin(th) * dy);
            delta_y = (sin(th) * dx + cos(th) * dy);

            x += delta_x;
            y += delta_y;
        }
        th = imu_heading; //using imu directly

        //ROS_INFO_STREAM( "delta_th: "<< delta_th << ", dt: " << dt <<", v_th: " << vth <<", current heading: " <<th);

        publish_odom(publisher_odom, tf_odom_baselink_broadcaster, current_time, x, y, th, vxy, vth);
        //ROS_INFO_STREAM( _imu_timestamp << ", v_th: " << vth <<", current heading: " <<th);

        //remember something
        last_time_odom = current_time;
        last_left_distance = v_odom_data.at(0);
        last_right_distance = v_odom_data.at(1);


        //ultrasound
        publish_ultrasound(publisher_ultrasound, tf_baselink_ultrasound_broadcaster);

        //imu
        publish_imu(publisher_imu, tf_baselink_imulink_broadcaster);

    }

    mobileBase.disConnect();

    return 0;
}


void velocity_callback(const geometry_msgs::Twist &command) {
    double linear_x = command.linear.x;
    double angular_z = command.angular.z;

    double v_left;
    double v_right;
    // Compute wheels velocities:
    if (linear_x == 0.0) {
        v_right = angular_z * wheel_track / 2.0;
        v_left = -v_right;
    } else if (angular_z == 0.0) {
        v_left = v_right = linear_x;
    } else {
        v_left = linear_x - angular_z * wheel_track / 2.0;
        v_right = linear_x + angular_z * wheel_track / 2.0;
    }
    mobileBase.setWheelSpeed({{1,v_left}, {2, v_right}});
}

void publish_odom(ros::Publisher publisher_odom, tf::TransformBroadcaster tf_odom_baselink_broadcaster, ros::Time current_time, double x, double y, double th, double vxy, double vth){
    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    tf_odom_baselink_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";

    odom.twist.twist.linear.x = vxy;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = vth;

    /*
    odom.twist.twist.linear.x = delta_x / dt;
    odom.twist.twist.linear.y = delta_y / dt;
    odom.twist.twist.angular.z = vth;
    */
    //publish the message
    publisher_odom.publish(odom);
}

void publish_ultrasound(ros::Publisher publisher_ultrasound, tf::TransformBroadcaster tf_baselink_ultrasound_broadcaster) {

    ros::Time current_time = ros::Time::now();
    //first, we'll publish the transform over tf

    tf_baselink_ultrasound_broadcaster.sendTransform(
            tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                                 ros::Time::now(),
                                 "base_link",
                                 "ultrasound"));

    //next, we send the ultrasound pointCloud

    //The 6 sensors are 1:front-left 2:front-middle 3:front-right 4:rear-right 5: rear-left 6:not used
    std::vector<double> range_ultrasound = mobileBase.getRangeSensor();
    if(range_ultrasound.empty()){
        return;
    }


    unsigned long sensor_total_num = ultrasound_used.size(); //how many sensors used

    sensor_msgs::PointCloud cloud;
    cloud.header.stamp = current_time;
    cloud.header.frame_id = "ultrasound";
    cloud.points.resize(sensor_total_num);

    //we'll also add an intensity channel to the cloud
    cloud.channels.resize(1);
    cloud.channels[0].name = "intensities";
    cloud.channels[0].values.resize(sensor_total_num);

    int i =0; //pointCloud index.
    for (auto it =ultrasound_used.begin(); it!=ultrasound_used.end();it++) {
        int sensor_index = *it;

        //adding offset related to center of the mobile base.
        if(range_ultrasound[sensor_index] == 0.0 || range_ultrasound[sensor_index]> ultrasound_max_valid_range){ //invalid data
            range_ultrasound[sensor_index] = 10000.0; //set to a big enough value
        } else {
            range_ultrasound[sensor_index] = ultrasound_offsets[sensor_index] + range_ultrasound[sensor_index]; //append the offset
        }

        //adding to cloud
        cloud.points[i].x = range_ultrasound[sensor_index] * cos(ultrasound_angles[sensor_index] *_degree_to_radian_factor);
        cloud.points[i].y = range_ultrasound[sensor_index] * sin(ultrasound_angles[sensor_index] *_degree_to_radian_factor);
        cloud.points[i].z = 0.2; //fixed height 0.2m
        cloud.channels[0].values[i] = 1.0;

        i++;

    }

    publisher_ultrasound.publish(cloud);

}

void publish_imu(ros::Publisher publisher_imu, tf::TransformBroadcaster tf_baselink_imulink_broadcaster) {
    ros::Time current_time = ros::Time::now();
    //first, we'll publish the transform over tf

    tf_baselink_imulink_broadcaster.sendTransform(
            tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
                                 ros::Time::now(),
                                 "base_link",
                                 "imu_link"));

    //next, we send the imu

    std::vector<double> v_imu = mobileBase.getIMU();
    if(v_imu.empty()){
        return;
    }
    //check time_stamp
    //validate the imu update timestamp
    if(_imu_timestamp!=0.0 && v_imu.at(0) > (_imu_timestamp + 1000)){
        ROS_ERROR_STREAM( "IMU is not updated in 1000ms.\n");
    }

    sensor_msgs::Imu imu;
    imu.header.stamp = current_time;
    imu.header.frame_id = "imu_link";

    imu.angular_velocity.x = v_imu.at(1);
    imu.angular_velocity.y = v_imu.at(2);
    imu.angular_velocity.z = v_imu.at(3);

    imu.orientation.w = v_imu.at(4);
    imu.orientation.x = v_imu.at(5);
    imu.orientation.y = v_imu.at(6);
    imu.orientation.z = v_imu.at(7);

    imu.linear_acceleration.x =v_imu.at(8);
    imu.linear_acceleration.y =v_imu.at(9);
    imu.linear_acceleration.z =v_imu.at(10);

    publisher_imu.publish(imu);
}