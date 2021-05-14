#include "loop_planner.hpp"

loop_planner::loop_planner()
{
    nh.getParam("/x_min_yas", x_min_yas);
    nh.getParam("/x_max_yas", x_max_yas);
    nh.getParam("/x_min_abb", x_min_abb);
    nh.getParam("/x_max_abb", x_max_abb);
    nh.getParam("/loop_min", loop_min);
    nh.getParam("/loop_max", loop_max);

    sub_yas_tcp = nh.subscribe<geometry_msgs::Point>("/yas_tcp_camera", 1, &loop_planner::tcp_yas_cb, this);

    sub_abb_tcp = nh.subscribe<geometry_msgs::Point>("/abb_tcp_camera", 1, &loop_planner::tcp_abb_cb, this);

    sub_edge = nh.subscribe<sensor_msgs::PointCloud2>("/extracted_edge_ts", 1, &loop_planner::edge_cb, this);
}
//including tcp values for yaskawa
void loop_planner::tcp_yas_cb(const geometry_msgs::Point input_yas)
{
    x_yas = input_yas.x;
    z_yas = input_yas.z;
}

//including tcp values for abb
void loop_planner::tcp_abb_cb(const geometry_msgs::Point input_abb)
{
    x_abb = input_abb.x;
    z_abb = input_abb.z;
}

void loop_planner::edge_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    x_avg = (x_yas + x_abb) / 2;
    x_avg_min = x_avg - 0.03;
    x_avg_max = x_avg + 0.03;//bounding box x values

    z_avg = (z_yas + z_abb) / 2;//bouding box z values

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_original(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*input, *edge_original);

    //extarcting points in the mid region of the extrated loop and storing as a vector 
    std::vector<std::vector<double>> mid_edge;
    for (std::size_t i = 0; i < edge_original->size(); ++i)
    {
        std::vector<double> *store_point = new std::vector<double>(3);
        (*store_point)[0] = (*edge_original)[i].x;
        (*store_point)[1] = (*edge_original)[i].y;
        (*store_point)[2] = (*edge_original)[i].z;
        if ((*store_point)[0] > x_avg_min && (*store_point)[0] < x_avg_max)
        {
            mid_edge.push_back(*store_point);
        }
        delete store_point;
    }

    //finding minimum and maximum y limits
    for (std::size_t i = 0; i < mid_edge.size(); ++i)
    {
        double *temp_y = new double;
        *temp_y = mid_edge[i][1];
        if (i == 0)
        {
            y_min = *temp_y;
            y_max = *temp_y;
            delete temp_y;
            continue;
        }
        if (*temp_y < y_min)
        {
            y_min = *temp_y;
            delete temp_y;
            continue;
        }
        if (*temp_y > y_max)
        {
            y_max = *temp_y;
            delete temp_y;
            continue;
        }
        delete temp_y;
    }

    y_dist = y_max - y_min;
    
    //printing commands as per loop bouding box parameters
    if (z_yas > 0.58 || z_abb > 0.58)
    {
        if (x_yas < x_min_yas)
        {
            std::cout << "Push Yaskawa towards the center" << std::endl;
        }
        else if (x_yas > x_max_yas)
        {
            std::cout << "Pull Yaskawa away from the center" << std::endl;
        }
        else if (x_abb < x_min_abb)
        {
            std::cout << "Pull ABB away from the center" << std::endl;
        }
        else if (x_abb > x_max_abb)
        {
            std::cout << "Push ABB towards the center" << std::endl;
        }
        else if (y_dist > loop_max)
        {
            std::cout << "Pull robots away from each other" << std::endl;
        }
        else if (y_dist < loop_min)
        {
            std::cout << "Push robots towards each other" << std::endl;
        }
        else
        {
            std::cout << "Loop is good for insertion" << std::endl;
        }
    }
    
    else
    {
        std::cout << "Loop is good for insertion" << std::endl;
    }
}

int main(int argc, char **argv)
{
    // Initialize ROS
    ROS_INFO("Starting planning node");
    ros::init(argc, argv, "loop_detection_original");

    loop_planner x_extract;

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
