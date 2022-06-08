/*
 * @Description: gps_localization
 * @Author: cyh
 * @Date: 2022-06-06 
 */
#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include "LocalCartesian.hpp"
#include "tf2/LinearMath/Quaternion.h"

struct my_pose
{
    double latitude;
    double longitude;
    double altitude;
};
//角度制转弧度制
double rad(double d) 
{
	return d * 3.1415926 / 180.0;
}
//全局变量
static double EARTH_RADIUS = 6378.137;//地球半径
ros::Publisher state_pub_;
ros::Publisher pose_pub_;
ros::Publisher gps_odom_;
ros::Subscriber imu_sub_;
nav_msgs::Path ros_path_;
geometry_msgs::PoseStamped gps_pose_;
bool init;
my_pose init_pose;
GeographicLib::LocalCartesian geoConverter;
double xyz[3];
sensor_msgs::ImuConstPtr imu_msg_ptr_(new sensor_msgs::Imu);
std::ofstream out_file("/home/cyh/gnss.txt", std::ios::out);

 void imuCallback(const sensor_msgs::ImuConstPtr& imu_msg_ptr){
     imu_msg_ptr_=imu_msg_ptr;
 } 


void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr)
{
    //初始化
    if(!init)
    {
        //geoConverter.Reset(48.9852369622, 8.39364145644, 116.372428894); // kitti
        // geoConverter.Reset(22.338715725, 114.263613048, 96.07); //hk
         geoConverter.Reset(29.4943633258, 106.567516844, 234.04);
         //geoConverter.Reset(29.4943856318, 106.567528749, 234.14); // 6.3
         //geoConverter.Reset(29.49436884, 106.567526548, 234.03); // 6.6
         //geoConverter.Reset(40.663537015, -74.56454322, 44.26); // park
         //geoConverter.Reset(29.49249408, 106.56778832, 228.92);
        //geoConverter.Reset(29.4913837447, 106.564610465, 213.34); // jiaxiao
        //geoConverter.Reset(29.4912867154, 106.564572946, 213.39);
        init = true;
    }

    geoConverter.Forward(gps_msg_ptr->latitude, gps_msg_ptr->longitude, gps_msg_ptr->altitude, xyz[0], xyz[1], xyz[2]);
    // geoConverter.Forward(gps_msg_ptr->latitude, gps_msg_ptr->longitude, 0, xyz[0], xyz[1], xyz[2]);

   
    double x = xyz[0];
    double y = xyz[1];
    double z = xyz[2];

    out_file << x << "," << y << "," << z << "\n";

    //发布gps_pose
    gps_pose_.header.frame_id = "map";
    gps_pose_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose1;
    pose1.header = gps_pose_.header;
    pose1.pose.position.x = x;
    pose1.pose.position.y = y;
    pose1.pose.position.z = z;


    pose1.pose.orientation.x = imu_msg_ptr_->orientation.x;
    pose1.pose.orientation.y = imu_msg_ptr_->orientation.y;
    pose1.pose.orientation.z = imu_msg_ptr_->orientation.z;
    pose1.pose.orientation.w = imu_msg_ptr_->orientation.w;


    //ROS_INFO("( x:%0.6f ,y:%0.6f ,z:%0.6f)",x ,y ,z );

    pose_pub_.publish(pose1);

    // publish gps_odometry
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "map";
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header = odom_msg.header;
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = z;
    odom_msg.pose.pose.orientation.x = imu_msg_ptr_->orientation.x;
    odom_msg.pose.pose.orientation.y = imu_msg_ptr_->orientation.y;
    odom_msg.pose.pose.orientation.z = imu_msg_ptr_->orientation.z;
    odom_msg.pose.pose.orientation.w = imu_msg_ptr_->orientation.w;
    gps_odom_.publish(odom_msg);




    //发布轨迹
    ros_path_.header.frame_id = "map";
    ros_path_.header.stamp = ros::Time::now();  

    geometry_msgs::PoseStamped pose;
    pose.header = ros_path_.header;

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    ros_path_.poses.push_back(pose);

    //ROS_INFO("( x:%0.6f ,y:%0.6f ,z:%0.6f)",x ,y ,z );

    state_pub_.publish(ros_path_);
    

}


    



int main(int argc,char **argv)
{
    init = false;
    ros::init(argc,argv,"gps_subscriber");
    ros::NodeHandle n;
    ros::Subscriber pose_sub=n.subscribe("/gps_driver",10,gpsCallback);
        
    state_pub_ = n.advertise<nav_msgs::Path>("/gps_path", 10);
    pose_pub_ = n.advertise<geometry_msgs::PoseStamped> ("/gps_pose", 10);
    imu_sub_= n.subscribe("/imu_correct", 10, imuCallback);
    gps_odom_ = n.advertise<nav_msgs::Odometry> ("/gps_odom", 10);


    ros::spin();
    return 0;
}

