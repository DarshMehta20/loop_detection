#ifndef LOOP_PLANNER_HPP
#define LOOP_PLANNER_HPP

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include <iostream>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>

//timer
#include <vector>
#include <ctime>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <time.h>

// Transform
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Point.h>
 
class loop_planner
{
private:
    ros::Publisher pub1;
    // ros::Publisher pub2;
    ros::Subscriber sub_edge;
    ros::Subscriber sub_yas_tcp;
    ros::Subscriber sub_abb_tcp;
    ros::NodeHandle nh;

    double x_min_yas, x_max_yas, x_min_abb, x_max_abb, loop_min, loop_max;
    double x_yas, x_abb, z_yas, z_abb, z_avg, x_avg, x_avg_min, x_avg_max, y_min, y_max, y_dist, time_pcl;

public:
    loop_planner();

    void tcp_yas_cb(const geometry_msgs::Point input_yas);
    void tcp_abb_cb(const geometry_msgs::Point input_abb);

    void edge_cb(const sensor_msgs::PointCloud2ConstPtr &input);

};
 
#endif