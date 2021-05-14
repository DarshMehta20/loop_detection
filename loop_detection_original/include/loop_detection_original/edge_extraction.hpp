#ifndef EDGE_EXTRACTION_HPP
#define EDGE_EXTRACTION_HPP

#include <ros/ros.h>
#include <iostream>

// PCL specific includes
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
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
 
class edge_extract
{
private:
    ros::Publisher pub1;
    ros::Publisher pub2;
    ros::Subscriber sub;
    ros::Subscriber sub_yas_tcp;
    ros::Subscriber sub_abb_tcp;
    ros::NodeHandle nh;

    float xmin, xmax, ymin, ymax, zmin, zmax, initialradius, clusterradius, voxel_grid_size;
    int ext_points, int_points;
    double x_yas, y_yas, z_yas, x_abb, y_abb, z_abb;
    double time_pcl;

public:
    edge_extract();

    void tcp_yas_cb(const geometry_msgs::Point input_yas);
    void tcp_abb_cb(const geometry_msgs::Point input_abb);

    void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input);

};
 
#endif