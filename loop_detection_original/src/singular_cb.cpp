#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
// #include <pcl-1.8/pcl/filters/passthrough.h>
// #include <pcl-1.8/pcl/ModelCoefficients.h>
// #include <pcl-1.8/pcl/point_types.h>
// #include <pcl-1.8/pcl/io/pcd_io.h>
// #include <pcl-1.8/pcl/io/ply_io.h>
// #include <pcl-1.8/pcl/kdtree/kdtree.h>
// #include <pcl-1.8/pcl/segmentation/extract_clusters.h>

// #include <pcl-1.8/pcl/ModelCoefficients.h>
// #include <pcl-1.8/pcl/filters/extract_indices.h>
// #include <pcl-1.8/pcl/filters/voxel_grid.h>
// #include <pcl-1.8/pcl/features/normal_3d.h>
// #include <pcl-1.8/pcl/sample_consensus/method_types.h>
// #include <pcl-1.8/pcl/sample_consensus/model_types.h>
// #include <pcl-1.8/pcl/segmentation/sac_segmentation.h>
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

ros::Publisher pub1;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
bool flag;

bool sortcol_xy(const std::vector<double> &v1,
                const std::vector<double> &v2)
{
  return v1[3] < v2[3];
}

void cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{

  if (!flag)
  {


    //timer start
    std::clock_t start, end;
    start = clock();

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud_original;
    pcl::fromROSMsg(*input, cloud_original);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //hardcoded tshirt extraxtion
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_original.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.8, 1.05);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-0.4, 0.4);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-0.18, 0.15);
    pass.filter(*cloud_filtered);

    //timer stop
    end = clock();
    double time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    std::cout << "Time taken by pass through filter is : " << std::fixed
              << time_taken << std::setprecision(7);
    std::cout << " sec " << std::endl;

    // pcl::PCLPointCloud2* cloud_output1 = new pcl::PCLPointCloud2;
    // pcl::PCLPointCloud2ConstPtr cloudPtr1(cloud_output1);
    // pcl::toPCLPointCloud2(*cloud_filtered, *cloud_output1);
    // sensor_msgs::PointCloud2 filtered;
    // pcl_conversions::fromPCL(*cloud_output1, filtered);
    // filtered.is_bigendian = false;
    // filtered.header.seq=1;
    // filtered.header.stamp=ros::Time::now();
    // filtered.header.frame_id=cloud_original.header.frame_id;
    // // output.height = cloud_filtered.height;
    // // output.width = cloud_filtered.width;
    // pub1.publish (filtered);

    //Voxelization
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(0.001f, 0.001f, 0.001f);
    vg.filter(*cloud_filtered);

    pcl::PCLPointCloud2 *cloud_output11 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr11(cloud_output11);
    pcl::toPCLPointCloud2(*cloud_filtered, *cloud_output11);
    sensor_msgs::PointCloud2 filtered_voxel;
    pcl_conversions::fromPCL(*cloud_output11, filtered_voxel);
    filtered_voxel.is_bigendian = false;
    filtered_voxel.header.seq = 1;
    filtered_voxel.header.stamp = ros::Time::now();
    filtered_voxel.header.frame_id = cloud_original.header.frame_id;
    // output.height = cloud_filtered.height;
    // output.width = cloud_filtered.width;
    pub2.publish(filtered_voxel);

    //generating a secondary 2d pt-cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZ>);

    *cloud = *cloud_filtered;
    *cloud_xy = *cloud;

    for (std::size_t i = 0; i < cloud_xy->size(); ++i)
    {
      (*cloud_xy)[i].z = 0;
    }

    pcl::PCLPointCloud2 *cloud_output3 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr3(cloud_output3);
    pcl::toPCLPointCloud2(*cloud_xy, *cloud_output3);
    sensor_msgs::PointCloud2 segmented_ts_xy;
    pcl_conversions::fromPCL(*cloud_output3, segmented_ts_xy);
    segmented_ts_xy.is_bigendian = false;
    segmented_ts_xy.header.seq = 1;
    segmented_ts_xy.header.stamp = ros::Time::now();
    segmented_ts_xy.header.frame_id = cloud_original.header.frame_id;
    //segmented_ts.height = cloud_cluster_extracted_ts.height;
    //segmented_ts.width = cloud_cluster_extracted_ts.width;
    pub3.publish(segmented_ts_xy);

    //kd tree search for 2d pt-cl
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_xy);
    pcl::PointXYZ searchPoint;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = 0.003; //search radius
    std::vector<std::vector<int>> I_2d;

    start = clock();
    // Kd tree radius search
    for (std::size_t i = 0; i < cloud_xy->size(); ++i)
    {
      searchPoint.x = (*cloud_xy)[i].x;
      searchPoint.y = (*cloud_xy)[i].y;
      searchPoint.z = (*cloud_xy)[i].z;
      if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
      {
        I_2d.push_back(pointIdxRadiusSearch);
      }
    }
    end = clock();
    time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    std::cout << "Time taken by kdtree search 1 is : " << std::fixed
              << time_taken << std::setprecision(7);
    std::cout << " sec " << std::endl;

    std::vector<int> ncols_2d;
    for (std::size_t i = 0; i < I_2d.size(); i++)
    {
      std::size_t a = I_2d[i].size();
      ncols_2d.push_back(a);
    }

    //index vectors for internal and external points
    std::vector<int> int_1_idx;  // useless pts
    std::vector<int> edge_1_idx; //outer edge
    std::vector<int> edge_2_idx; //inner cluster

    for (std::size_t i = 0; i < ncols_2d.size(); i++)
    {
      if (ncols_2d[i] < 5)
      {
        edge_1_idx.push_back(i);
      }
      else if (ncols_2d[i] > 6)
      {
        edge_2_idx.push_back(i);
      }
      else
      {
        int_1_idx.push_back(i);
      }
    }

    std::vector<std::vector<double>> mat_top3d_noNAN_vector;
    for (std::size_t i = 0; i < edge_1_idx.size(); i++)
    {
      std::vector<double> *store_point = new std::vector<double>(4);
      (*store_point)[0] = (*cloud)[edge_1_idx[i]].x;
      (*store_point)[1] = (*cloud)[edge_1_idx[i]].y;
      (*store_point)[2] = (*cloud)[edge_1_idx[i]].z;
      (*store_point)[3] = 0;
      mat_top3d_noNAN_vector.push_back(*store_point);
      delete store_point;
    }

    start = clock();
    //search and sort in increasing x and decreasing y
    for (std::size_t i = 0; i < mat_top3d_noNAN_vector.size(); ++i)
    {
      mat_top3d_noNAN_vector[i][0] = (floor(mat_top3d_noNAN_vector[i][0] / 0.005) + 1) * 0.005;
      mat_top3d_noNAN_vector[i][3] = 1000 * mat_top3d_noNAN_vector[i][0] - mat_top3d_noNAN_vector[i][1];
    }

    sort(mat_top3d_noNAN_vector.begin(), mat_top3d_noNAN_vector.end(), sortcol_xy);

    std::vector<std::vector<double>> mat_top3d_maxy_vector;
    for (std::size_t i = 0; i < mat_top3d_noNAN_vector.size(); i++)
    {
      std::vector<double> *store_point = new std::vector<double>(3);
      (*store_point)[0] = mat_top3d_noNAN_vector[i][0];
      (*store_point)[1] = mat_top3d_noNAN_vector[i][1];
      (*store_point)[2] = mat_top3d_noNAN_vector[i][2];
      if (i == 0)
      {
        mat_top3d_maxy_vector.push_back(*store_point);
        delete store_point;
        continue;
      }
      if (std::abs((*store_point)[0] - mat_top3d_maxy_vector[mat_top3d_maxy_vector.size() - 1][0]) < 0.0001)
      {
        if ((*store_point)[1] < mat_top3d_maxy_vector[mat_top3d_maxy_vector.size() - 1][1])
        {
          mat_top3d_maxy_vector[mat_top3d_maxy_vector.size() - 1] = *store_point;
        }
        delete store_point;
        continue;
      }
      else
      {
        mat_top3d_maxy_vector.push_back(*store_point);
        delete store_point;
        continue;
      }
      delete store_point;
    }
    end = clock();
    time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    std::cout << "Time taken by top edge detection is : " << std::fixed
              << time_taken << std::setprecision(7);
    std::cout << " sec " << std::endl;

    //internal clusters edge removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr mat_edge3d_noNAN(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::size_t i = 0; i < edge_2_idx.size(); i++)
    {
      mat_edge3d_noNAN->push_back((*cloud)[edge_2_idx[i]]);
    }
    mat_edge3d_noNAN->width = mat_edge3d_noNAN->size();
    mat_edge3d_noNAN->height = 1;
    mat_edge3d_noNAN->is_dense = true;

    pcl::PointCloud<pcl::PointXYZ>::Ptr mat_edge2d_noNAN(new pcl::PointCloud<pcl::PointXYZ>);
    *mat_edge2d_noNAN = *mat_edge3d_noNAN;
    for (std::size_t i = 0; i < mat_edge2d_noNAN->size(); ++i)
    {
      (*mat_edge2d_noNAN)[i].z = 0;
    }

    start = clock();
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_edge2d;
    kdtree_edge2d.setInputCloud(mat_edge2d_noNAN);
    pcl::PointXYZ searchPoint_edge2d;

    std::vector<int> pointIdxRadiusSearch_edge2d;
    std::vector<float> pointRadiusSquaredDistance_edge2d;

    float radius_edge2d = 0.003;
    std::vector<std::vector<int>> I_edge2d;

    for (std::size_t i = 0; i < mat_edge2d_noNAN->size(); ++i)
    {
      searchPoint_edge2d.x = (*mat_edge2d_noNAN)[i].x;
      searchPoint_edge2d.y = (*mat_edge2d_noNAN)[i].y;
      searchPoint_edge2d.z = (*mat_edge2d_noNAN)[i].z;

      if (kdtree_edge2d.radiusSearch(searchPoint_edge2d, radius, pointIdxRadiusSearch_edge2d, pointRadiusSquaredDistance_edge2d) > 0)
      {
        I_edge2d.push_back(pointIdxRadiusSearch_edge2d);
      }
    }

    std::vector<int> ncols_edge2d;
    for (std::size_t i = 0; i < I_edge2d.size(); i++)
    {
      std::size_t a = I_edge2d[i].size();
      ncols_edge2d.push_back(a);
    }

    std::vector<int> edge_removed_noise_point;
    for (std::size_t i = 0; i < ncols_edge2d.size(); i++)
    {
      if (ncols_edge2d[i] > 3)
      {
        edge_removed_noise_point.push_back(i);
      }
    }
    end = clock();
    time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    std::cout << "Time taken by kdflann and noise removal is : " << std::fixed
              << time_taken << std::setprecision(7);
    std::cout << " sec " << std::endl;

    start = clock();
    std::vector<std::vector<double>> mat_edge3d_vector;
    for (std::size_t i = 0; i < edge_removed_noise_point.size(); i++)
    {
      std::vector<double> *store_point = new std::vector<double>(4);
      (*store_point)[0] = (*mat_edge3d_noNAN)[edge_removed_noise_point[i]].x;
      (*store_point)[1] = (*mat_edge3d_noNAN)[edge_removed_noise_point[i]].y;
      (*store_point)[2] = (*mat_edge3d_noNAN)[edge_removed_noise_point[i]].z;
      (*store_point)[3] = 0;
      mat_edge3d_vector.push_back(*store_point);
      delete store_point;
    }

    //search and sort in increasing x and decreasing y
    for (std::size_t i = 0; i < mat_edge3d_vector.size(); ++i)
    {
      mat_edge3d_vector[i][0] = (floor(mat_edge3d_vector[i][0] / 0.005) + 1) * 0.005;
      mat_edge3d_vector[i][3] = 1000 * mat_edge3d_vector[i][0] + mat_edge3d_vector[i][1];
    }

    sort(mat_edge3d_vector.begin(), mat_edge3d_vector.end(), sortcol_xy);

    std::vector<std::vector<double>> mat_edge3d_cluster_vector;
    for (std::size_t i = 0; i < mat_edge3d_vector.size(); i++)
    {
      std::vector<double> *store_point = new std::vector<double>(3);
      (*store_point)[0] = mat_edge3d_vector[i][0];
      (*store_point)[1] = mat_edge3d_vector[i][1];
      (*store_point)[2] = mat_edge3d_vector[i][2];
      if (i == 0)
      {
        mat_edge3d_cluster_vector.push_back(*store_point);
        delete store_point;
        continue;
      }
      if (std::abs((*store_point)[0] - mat_edge3d_cluster_vector[mat_edge3d_cluster_vector.size() - 1][0]) < 0.0001)
      {
        if (std::abs((*store_point)[1] - mat_edge3d_cluster_vector[mat_edge3d_cluster_vector.size() - 1][1]) < 0.01)
        {
          mat_edge3d_cluster_vector.push_back(*store_point);
        }
        delete store_point;
        continue;
      }
      else
      {
        mat_edge3d_cluster_vector.push_back(*store_point);
        delete store_point;
        continue;
      }
      delete store_point;
    }
    end = clock();
    time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    std::cout << "Time taken by top cluster extraction is : " << std::fixed
              << time_taken << std::setprecision(7);
    std::cout << " sec " << std::endl;

    pcl::PointCloud<pcl::PointXYZ>::Ptr mat_edge3d_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    mat_edge3d_cluster->width = mat_edge3d_cluster_vector.size();
    mat_edge3d_cluster->height = 1;
    mat_edge3d_cluster->points.resize(mat_edge3d_cluster->width * mat_edge3d_cluster->height);
    for (std::size_t i = 0; i < mat_edge3d_cluster->size(); ++i)
    {
      (*mat_edge3d_cluster)[i].x = mat_edge3d_cluster_vector[i][0];
      (*mat_edge3d_cluster)[i].y = mat_edge3d_cluster_vector[i][1];
      (*mat_edge3d_cluster)[i].z = mat_edge3d_cluster_vector[i][2];
    }

    start = clock();
    std::vector<std::vector<double>> mat_edge3d_maxz_vector;
    for (std::size_t i = 0; i < mat_edge3d_cluster_vector.size(); i++)
    {
      std::vector<double> *store_point = new std::vector<double>(3);
      (*store_point)[0] = mat_edge3d_cluster_vector[i][0];
      (*store_point)[1] = mat_edge3d_cluster_vector[i][1];
      (*store_point)[2] = mat_edge3d_cluster_vector[i][2];
      if (i == 0)
      {
        mat_edge3d_maxz_vector.push_back(*store_point);
        delete store_point;
        continue;
      }
      if (std::abs((*store_point)[0] - mat_edge3d_maxz_vector[mat_edge3d_maxz_vector.size() - 1][0]) < 0.0001)
      {
        if ((*store_point)[2] < mat_edge3d_maxz_vector[mat_edge3d_maxz_vector.size() - 1][2])
        {
          mat_edge3d_maxz_vector[mat_edge3d_maxz_vector.size() - 1] = *store_point;
        }
        delete store_point;
        continue;
      }
      else
      {
        mat_edge3d_maxz_vector.push_back(*store_point);
        delete store_point;
        continue;
      }
      delete store_point;
    }
    end = clock();
    time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    std::cout << "Time taken by loop min z is : " << std::fixed
              << time_taken << std::setprecision(7);
    std::cout << " sec " << std::endl;


    start = clock();
    //combining external top edge and internal loop edge
    std::vector<std::vector<double>> mat_edge_total_vector;
    mat_edge_total_vector = mat_top3d_maxy_vector;
    mat_edge_total_vector.insert(mat_edge_total_vector.end(), mat_edge3d_maxz_vector.begin(), mat_edge3d_maxz_vector.end());

    pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_edge(new pcl::PointCloud<pcl::PointXYZ>);
    extracted_edge->width = mat_edge_total_vector.size();
    extracted_edge->height = 1;
    extracted_edge->points.resize(extracted_edge->width * extracted_edge->height);

    for (std::size_t i = 0; i < extracted_edge->size(); ++i)
    {
      (*extracted_edge)[i].x = mat_edge_total_vector[i][0];
      (*extracted_edge)[i].y = mat_edge_total_vector[i][1];
      (*extracted_edge)[i].z = mat_edge_total_vector[i][2];
    }
    end = clock();
    time_taken = double(end - start) / double(CLOCKS_PER_SEC);
    std::cout << "Time taken by combining both loops is : " << std::fixed
              << time_taken << std::setprecision(7);
    std::cout << " sec " << std::endl;

    pcl::PCLPointCloud2 *cloud_output4 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr4(cloud_output4);
    pcl::toPCLPointCloud2(*extracted_edge, *cloud_output4);
    sensor_msgs::PointCloud2 extracted_edge_ts;
    pcl_conversions::fromPCL(*cloud_output4, extracted_edge_ts);
    extracted_edge_ts.is_bigendian = false;
    extracted_edge_ts.header.seq = 1;
    extracted_edge_ts.header.stamp = ros::Time::now();
    extracted_edge_ts.header.frame_id = cloud_original.header.frame_id;
    //segmented_ts.height = cloud_cluster_extracted_ts.height;
    //segmented_ts.width = cloud_cluster_extracted_ts.width;
    pub4.publish(extracted_edge_ts);

    

    flag = true;
  }
}

////////////////////////////////////MAIN FUNCTION

int main(int argc, char **argv)
{
  // Initialize ROS
  ROS_INFO("Starting processing node");
  ros::init(argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;

  ros::Rate loop_rate(30);

  // //Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points", 1, cloud_cb);

  ROS_INFO("Starting topic /filtered_voxel");
  pub2 = nh.advertise<sensor_msgs::PointCloud2>("filtered_voxel", 1);

  ROS_INFO("Starting topic /segmented_ts_xy");
  pub3 = nh.advertise<sensor_msgs::PointCloud2>("segmented_ts_xy", 1);

  ROS_INFO("Starting topic /extracted_edge_ts");
  pub4 = nh.advertise<sensor_msgs::PointCloud2>("extracted_edge_ts", 1);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
