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

class Subscribe_And_Publish
{
private:
  ros::Publisher pub1;
  ros::Publisher pub11;
  ros::Publisher pub2;
  ros::Publisher pub3;
  ros::Publisher pub4;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::NodeHandle nh;

  bool sortcol_xy(const std::vector<double> &v1,
                  const std::vector<double> &v2)
  {
    return v1[3] < v2[3];
  }

public:
  Subscribe_And_Publish()
  {
    sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth/color/points", 1, &Subscribe_And_Publish::callback, this);

    pub11 = nh.advertise<sensor_msgs::PointCloud2>("filtered_voxel", 1);
  }

  void callback(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    ROS_INFO("111");
    std::cout << "1" << std::endl;
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud_original;
    std::cout << "2" << std::endl;
    pcl::fromROSMsg(*input, cloud_original);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_original.makeShared());
    pass.setFilterFieldName("z");
    // pass.setFilterLimits(-1.5, 0.0);
    pass.setFilterLimits(0.0, 1.05);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x");
    // pass.setFilterLimits(-0.4, 0.4);
    pass.setFilterLimits(-0.4, 0.4);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    // pass.setFilterLimits(-2, 0.06);
    pass.setFilterLimits(-0.2, 0.2);
    pass.filter(*cloud_filtered);

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
    pub11.publish(filtered_voxel);

    //   // pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    //   // tree->setInputCloud(cloud_filtered);

    //   // std::vector<pcl::PointIndices> cluster_indices;
    //   // cluster_indices.size();
    //   // pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    //   // std::cout<<"1"<<std::endl;
    //   // ec.setClusterTolerance(0.005); // 5mm
    //   // ec.setMinClusterSize(10000);
    //   // ec.setMaxClusterSize(75000);
    //   // ec.setSearchMethod(tree);
    //   // ec.setInputCloud(cloud_filtered);
    //   // std::cout<<"2"<<std::endl;
    //   // ec.extract(cluster_indices);
    //   // std::cout<<"3"<<std::endl;
    //   // std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();
    //   // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster_extracted_ts(new pcl::PointCloud<pcl::PointXYZ>);
    //   // std::cout<<"4"<<std::endl;

    //   // try{
    //   //   if(cluster_indices.size() == 0){
    //   //     throw cluster_indices.size();
    //   //   }
    //   //   for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
    //   //   // std::cout<<"5"<<std::endl;
    //   //   cloud_cluster_extracted_ts->push_back((*cloud_filtered)[*pit]);
    //   //   }
    //   // }
    //   // catch(unsigned long x){
    //   //   std::cout<<"The size of the vecotr is zero"<<std::endl;

    //   // }

    //   // std::cout<<"6"<<std::endl;
    //   // cloud_cluster_extracted_ts->width = cloud_cluster_extracted_ts->size();
    //   // cloud_cluster_extracted_ts->height = 1;
    //   // cloud_cluster_extracted_ts->is_dense = true;

    //   // pcl::PCLPointCloud2* cloud_output2 = new pcl::PCLPointCloud2;
    //   // pcl::PCLPointCloud2ConstPtr cloudPtr2(cloud_output2);
    //   // pcl::toPCLPointCloud2(*cloud_cluster_extracted_ts, *cloud_output2);
    //   // sensor_msgs::PointCloud2 segmented_ts;
    //   // pcl_conversions::fromPCL(*cloud_output2, segmented_ts);
    //   // segmented_ts.is_bigendian = false;
    //   // segmented_ts.header.seq=1;
    //   // segmented_ts.header.stamp=ros::Time::now();
    //   // segmented_ts.header.frame_id=cloud_original.header.frame_id;
    //   // //segmented_ts.height = cloud_cluster_extracted_ts.height;
    //   // //segmented_ts.width = cloud_cluster_extracted_ts.width;
    //   // pub2.publish (segmented_ts);

    //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //   pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZ>);

    //   *cloud = *cloud_filtered;
    //   *cloud_xy = *cloud;

    //   for (std::size_t i = 0; i < cloud_xy->size(); ++i)
    //   {
    //       (*cloud_xy)[i].z = 0;
    //   }

    //   pcl::PCLPointCloud2* cloud_output3 = new pcl::PCLPointCloud2;
    //   pcl::PCLPointCloud2ConstPtr cloudPtr3(cloud_output3);
    //   pcl::toPCLPointCloud2(*cloud_xy, *cloud_output3);
    //   sensor_msgs::PointCloud2 segmented_ts_xy;
    //   pcl_conversions::fromPCL(*cloud_output3, segmented_ts_xy);
    //   segmented_ts_xy.is_bigendian = false;
    //   segmented_ts_xy.header.seq=1;
    //   segmented_ts_xy.header.stamp=ros::Time::now();
    //   segmented_ts_xy.header.frame_id=cloud_original.header.frame_id;
    //   //segmented_ts.height = cloud_cluster_extracted_ts.height;
    //   //segmented_ts.width = cloud_cluster_extracted_ts.width;
    //   pub3.publish (segmented_ts_xy);

    //   pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    //     kdtree.setInputCloud(cloud_xy);
    //     pcl::PointXYZ searchPoint;

    //     std::vector<int> pointIdxRadiusSearch;
    //     std::vector<float> pointRadiusSquaredDistance;

    //     float radius = 0.003;
    //     std::vector<std::vector<int>> I_2d;

    //     // Kd tree radius search
    //     for (std::size_t i = 0; i < cloud_xy->size(); ++i)
    //     {
    //         searchPoint.x = (*cloud_xy)[i].x;
    //         searchPoint.y = (*cloud_xy)[i].y;
    //         searchPoint.z = (*cloud_xy)[i].z;

    //         // std::cout << "Neighbors within radius search at (" << searchPoint.x
    //         //           << " " << searchPoint.y
    //         //           << " " << searchPoint.z
    //         //           << ") with radius=" << radius << std::endl;

    //         if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
    //         {
    //             I_2d.push_back(pointIdxRadiusSearch);
    //             // for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
    //             //     std::cout << "    " << (*cloud_xy)[pointIdxRadiusSearch[i]].x
    //             //               << " " << (*cloud_xy)[pointIdxRadiusSearch[i]].y
    //             //               << " " << (*cloud_xy)[pointIdxRadiusSearch[i]].z
    //             //               << std::endl;
    //         }
    //     }

    //     std::vector<int> ncols_2d;
    //     for (std::size_t i = 0; i < I_2d.size(); i++)
    //     {
    //         std::size_t a = I_2d[i].size();
    //         ncols_2d.push_back(a);
    //     }
    //     // for (std::size_t i = 0; i < ncols_2d.size(); i++)
    //     // {
    //     //     std::cout << ncols_2d[i] << std::endl;
    //     // }

    //     std::vector<int> int_1_idx;
    //     std::vector<int> edge_1_idx;
    //     std::vector<int> edge_2_idx;

    //     for (std::size_t i = 0; i < ncols_2d.size(); i++)
    //     {
    //         if (ncols_2d[i] < 5)
    //         {
    //             edge_1_idx.push_back(i);
    //         }
    //         else if (ncols_2d[i] > 6)
    //         {
    //             edge_2_idx.push_back(i);
    //         }
    //         else
    //         {
    //             int_1_idx.push_back(i);
    //         }
    //     }

    //     // std::vector<std::vector<double>> mat_top3d_noNAN_vector;
    //     // for (std::size_t i = 0; i < edge_1_idx.size(); i++)
    //     // {
    //     //     std::vector<double> *store_point = new std::vector<double>(4);
    //     //     (*store_point)[0] = (*cloud)[edge_1_idx[i]].x;
    //     //     (*store_point)[1] = (*cloud)[edge_1_idx[i]].y;
    //     //     (*store_point)[2] = (*cloud)[edge_1_idx[i]].z;
    //     //     (*store_point)[3] = 0;
    //     //     mat_top3d_noNAN_vector.push_back(*store_point);
    //     //     delete store_point;
    //     // }

    //     std::vector<std::vector<double>> mat_top3d_noNAN_vector;
    //     for (std::size_t i = 0; i < cloud->size(); ++i)
    //     {
    //         std::vector<double> *store_point = new std::vector<double>(4);
    //         (*store_point)[0] = (*cloud)[i].x;
    //         (*store_point)[1] = (*cloud)[i].y;
    //         (*store_point)[2] = (*cloud)[i].z;
    //         (*store_point)[3] = 0;
    //         mat_top3d_noNAN_vector.push_back(*store_point);
    //         delete store_point;
    //     }

    //     for (std::size_t i = 0; i < mat_top3d_noNAN_vector.size(); ++i)
    //     {
    //         mat_top3d_noNAN_vector[i][0] = (floor(mat_top3d_noNAN_vector[i][0] / 0.005) + 1) * 0.005;
    //         // (*mat_top3d_noNAN)[i].x = (int)round(1000*(*mat_top3d_noNAN)[i].x);
    //         // (*mat_top3d_noNAN)[i].x = (double)(((*mat_top3d_noNAN)[i].x)/1000);
    //         mat_top3d_noNAN_vector[i][3] = 1000 * mat_top3d_noNAN_vector[i][0] - mat_top3d_noNAN_vector[i][1];
    //     }

    //     sort(mat_top3d_noNAN_vector.begin(), mat_top3d_noNAN_vector.end(), sortcol_xy);

    //     std::vector<std::vector<double>> mat_top3d_maxy_vector;
    //     for (std::size_t i = 0; i < mat_top3d_noNAN_vector.size(); i++)
    //     {
    //         std::vector<double> *store_point = new std::vector<double>(3);
    //         (*store_point)[0] = mat_top3d_noNAN_vector[i][0];
    //         (*store_point)[1] = mat_top3d_noNAN_vector[i][1];
    //         (*store_point)[2] = mat_top3d_noNAN_vector[i][2];
    //         if (i == 0)
    //         {
    //             mat_top3d_maxy_vector.push_back(*store_point);
    //             delete store_point;
    //             continue;
    //         }
    //         if (std::abs((*store_point)[0] - mat_top3d_maxy_vector[mat_top3d_maxy_vector.size() - 1][0]) < 0.0001)
    //         {
    //           //////////////////////chechk point 1
    //             if ((*store_point)[1] < mat_top3d_maxy_vector[mat_top3d_maxy_vector.size() - 1][1])
    //             {
    //                 mat_top3d_maxy_vector[mat_top3d_maxy_vector.size() - 1] = *store_point;
    //             }
    //             delete store_point;
    //             continue;
    //         }
    //         else
    //         {
    //             mat_top3d_maxy_vector.push_back(*store_point);
    //             delete store_point;
    //             continue;
    //         }
    //         delete store_point;
    //     }

    //     ///////////////////////////////// internal clusters edge removal

    //     pcl::PointCloud<pcl::PointXYZ>::Ptr mat_edge3d_noNAN(new pcl::PointCloud<pcl::PointXYZ>);
    //     for (std::size_t i = 0; i < edge_2_idx.size(); i++)
    //     {
    //         mat_edge3d_noNAN->push_back((*cloud)[edge_2_idx[i]]);
    //     }
    //     mat_edge3d_noNAN->width = mat_edge3d_noNAN->size();
    //     mat_edge3d_noNAN->height = 1;
    //     mat_edge3d_noNAN->is_dense = true;

    //     // std::cout << mat_edge3d_noNAN->size() << std::endl;

    //     pcl::PointCloud<pcl::PointXYZ>::Ptr mat_edge2d_noNAN(new pcl::PointCloud<pcl::PointXYZ>);
    //     *mat_edge2d_noNAN = *mat_edge3d_noNAN;
    //     for (std::size_t i = 0; i < mat_edge2d_noNAN->size(); ++i)
    //     {
    //         (*mat_edge2d_noNAN)[i].z = 0;
    //     }

    //     // std::cout << mat_edge2d_noNAN->size() << std::endl;
    //     // pcl::PLYWriter writer_edge;
    //     // writer_edge.write<pcl::PointXYZ>("mat_edge2d_noNAN.ply", *mat_edge2d_noNAN, false);

    //     pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_edge2d;
    //     kdtree_edge2d.setInputCloud(mat_edge2d_noNAN);
    //     pcl::PointXYZ searchPoint_edge2d;

    //     std::vector<int> pointIdxRadiusSearch_edge2d;
    //     std::vector<float> pointRadiusSquaredDistance_edge2d;

    //     float radius_edge2d = 0.003;
    //     std::vector<std::vector<int>> I_edge2d;

    //     // Kd tree radius search
    //     // for (std::size_t i = 0; i < mat_edge2d_noNAN->size(); ++i)
    //     for (std::size_t i = 0; i < mat_edge2d_noNAN->size(); ++i)
    //     {
    //         searchPoint_edge2d.x = (*mat_edge2d_noNAN)[i].x;
    //         searchPoint_edge2d.y = (*mat_edge2d_noNAN)[i].y;
    //         searchPoint_edge2d.z = (*mat_edge2d_noNAN)[i].z;

    //         // std::cout << "Neighbors within radius search at (" << searchPoint_edge2d.x
    //         //           << " " << searchPoint_edge2d.y
    //         //           << " " << searchPoint_edge2d.z
    //         //           << ") with radius=" << radius_edge2d << std::endl;

    //         if (kdtree_edge2d.radiusSearch(searchPoint_edge2d, radius, pointIdxRadiusSearch_edge2d, pointRadiusSquaredDistance_edge2d) > 0)
    //         {
    //             // std::cout << "Loop entered" << std::endl;
    //             // std::cout << pointIdxRadiusSearch_edge2d.size() << std::endl;
    //             I_edge2d.push_back(pointIdxRadiusSearch_edge2d);
    //             // for (std::size_t i = 0; i < pointIdxRadiusSearch_edge2d.size(); ++i)
    //             //     std::cout << "    " << (*mat_edge2d_noNAN)[pointIdxRadiusSearch_edge2d[i]].x
    //             //               << " " << (*mat_edge2d_noNAN)[pointIdxRadiusSearch_edge2d[i]].y
    //             //               << " " << (*mat_edge2d_noNAN)[pointIdxRadiusSearch_edge2d[i]].z
    //             //               << std::endl;
    //         }
    //         // std::cout << "Loop exitted" << std::endl;
    //     }
    //     // std::cout << "I_edge2d size is " << I_edge2d.size() << std::endl;

    //     // for (std::size_t i = 0; i < I_edge2d.size(); i++)
    //     // {
    //     //     for (std::size_t j = 0; j < I_edge2d[i].size(); j++)
    //     //     {
    //     //         std::cout << I_edge2d[i][j] << " , " << std::flush;
    //     //     }
    //     //     std::cout << std::endl;
    //     // }

    //     std::vector<int> ncols_edge2d;
    //     for (std::size_t i = 0; i < I_edge2d.size(); i++)
    //     {
    //         std::size_t a = I_edge2d[i].size();
    //         ncols_edge2d.push_back(a);
    //     }
    //     // std::cout << "ncols_edge2d size is " << ncols_edge2d.size() << std::endl;

    //     std::vector<int> edge_removed_noise_point;
    //     for (std::size_t i = 0; i < ncols_edge2d.size(); i++)
    //     {
    //         if (ncols_edge2d[i] > 3)
    //         {
    //             edge_removed_noise_point.push_back(i);
    //         }
    //     }
    //     // std::cout << "edge_removed_noise_point size is " << edge_removed_noise_point.size() << std::endl;

    //     std::vector<std::vector<double>> mat_edge3d_vector;
    //     for (std::size_t i = 0; i < edge_removed_noise_point.size(); i++)
    //     {
    //         std::vector<double> *store_point = new std::vector<double>(4);
    //         (*store_point)[0] = (*mat_edge3d_noNAN)[edge_removed_noise_point[i]].x;
    //         (*store_point)[1] = (*mat_edge3d_noNAN)[edge_removed_noise_point[i]].y;
    //         (*store_point)[2] = (*mat_edge3d_noNAN)[edge_removed_noise_point[i]].z;
    //         (*store_point)[3] = 0;
    //         mat_edge3d_vector.push_back(*store_point);
    //         delete store_point;
    //     }
    //     // std::cout << "Vector representing outside edge " << mat_edge3d_vector.size() << " data points." << std::endl;
    //     // std::cout << "Each vector has " << mat_edge3d_vector[0].size() << " data points." << std::endl;

    //     for (std::size_t i = 0; i < mat_edge3d_vector.size(); ++i)
    //     {
    //         mat_edge3d_vector[i][0] = (floor(mat_edge3d_vector[i][0] / 0.005) + 1) * 0.005;
    //         mat_edge3d_vector[i][3] = 1000 * mat_edge3d_vector[i][0] + mat_edge3d_vector[i][1];
    //     }

    //     sort(mat_edge3d_vector.begin(), mat_edge3d_vector.end(), sortcol_xy);

    //     // for (std::size_t i = 0; i < mat_edge3d_vector.size(); i++)
    //     // {
    //     //     for (std::size_t j = 0; j < mat_edge3d_vector[i].size(); j++)
    //     //     {
    //     //         std::cout << mat_edge3d_vector[i][j] << " , " << std::flush;
    //     //     }
    //     //     std::cout << std::endl;
    //     // }

    //     std::vector<std::vector<double>> mat_edge3d_cluster_vector;
    //     for (std::size_t i = 0; i < mat_edge3d_vector.size(); i++)
    //     {
    //         std::vector<double> *store_point = new std::vector<double>(3);
    //         (*store_point)[0] = mat_edge3d_vector[i][0];
    //         (*store_point)[1] = mat_edge3d_vector[i][1];
    //         (*store_point)[2] = mat_edge3d_vector[i][2];
    //         if (i == 0)
    //         {
    //             mat_edge3d_cluster_vector.push_back(*store_point);
    //             delete store_point;
    //             continue;
    //         }
    //         if (std::abs((*store_point)[0] - mat_edge3d_cluster_vector[mat_edge3d_cluster_vector.size() - 1][0]) < 0.0001)
    //         {
    //             if (std::abs((*store_point)[1] - mat_edge3d_cluster_vector[mat_edge3d_cluster_vector.size() - 1][1]) < 0.01)
    //             {
    //                 mat_edge3d_cluster_vector.push_back(*store_point);
    //             }
    //             delete store_point;
    //             continue;
    //         }
    //         // else if (std::abs((*store_point)[1] - mat_edge3d_cluster_vector[mat_edge3d_cluster_vector.size() - 1][1]) < 0.07)
    //         // {
    //         //     mat_edge3d_cluster_vector.push_back(*store_point);
    //         //     delete store_point;
    //         //     continue;
    //         // }
    //         else
    //         {
    //             mat_edge3d_cluster_vector.push_back(*store_point);
    //             delete store_point;
    //             continue;
    //         }
    //         delete store_point;
    //     }

    //     pcl::PointCloud<pcl::PointXYZ>::Ptr mat_edge3d_cluster(new pcl::PointCloud<pcl::PointXYZ>);
    //     mat_edge3d_cluster->width = mat_edge3d_cluster_vector.size();
    //     mat_edge3d_cluster->height = 1;
    //     mat_edge3d_cluster->points.resize(mat_edge3d_cluster->width * mat_edge3d_cluster->height);
    //     for (std::size_t i = 0; i < mat_edge3d_cluster->size(); ++i)
    //     {
    //         (*mat_edge3d_cluster)[i].x = mat_edge3d_cluster_vector[i][0];
    //         (*mat_edge3d_cluster)[i].y = mat_edge3d_cluster_vector[i][1];
    //         (*mat_edge3d_cluster)[i].z = mat_edge3d_cluster_vector[i][2];
    //     }
    //     // pcl::PLYWriter writer_cluster;
    //     // writer_cluster.write<pcl::PointXYZ>("mat_edge3d_cluster.ply", *mat_edge3d_cluster, false);

    //     std::vector<std::vector<double>> mat_edge3d_maxz_vector;
    //     for (std::size_t i = 0; i < mat_edge3d_cluster_vector.size(); i++)
    //     {
    //         std::vector<double> *store_point = new std::vector<double>(3);
    //         (*store_point)[0] = mat_edge3d_cluster_vector[i][0];
    //         (*store_point)[1] = mat_edge3d_cluster_vector[i][1];
    //         (*store_point)[2] = mat_edge3d_cluster_vector[i][2];
    //         if (i == 0)
    //         {
    //             mat_edge3d_maxz_vector.push_back(*store_point);
    //             delete store_point;
    //             continue;
    //         }
    //         if (std::abs((*store_point)[0] - mat_edge3d_maxz_vector[mat_edge3d_maxz_vector.size() - 1][0]) < 0.0001)
    //         {
    //           /////////////////////check point 2
    //             if ((*store_point)[2] < mat_edge3d_maxz_vector[mat_edge3d_maxz_vector.size() - 1][2])
    //             {
    //                 mat_edge3d_maxz_vector[mat_edge3d_maxz_vector.size() - 1] = *store_point;
    //             }
    //             delete store_point;
    //             continue;
    //         }
    //         // else if (std::abs((*store_point)[1] - mat_edge3d_maxz_vector[mat_edge3d_maxz_vector.size() - 1][1]) < 0.05)
    //         // {
    //         //     mat_edge3d_maxz_vector.push_back(*store_point);
    //         //     delete store_point;
    //         //     continue;
    //         // }
    //         else
    //         {
    //             mat_edge3d_maxz_vector.push_back(*store_point);
    //             delete store_point;
    //             continue;
    //         }
    //         delete store_point;
    //     }
    //     // std::cout << "Vector representing mat_edge3d_maxz_vector has " << mat_edge3d_maxz_vector.size() << " data points." << std::endl;
    //     // std::cout << "Each vector has " << mat_edge3d_maxz_vector[0].size() << " data points." << std::endl;

    //     std::vector<std::vector<double>> mat_edge_total_vector;
    //     mat_edge_total_vector = mat_top3d_maxy_vector;
    //     mat_edge_total_vector.insert(mat_edge_total_vector.end(), mat_edge3d_maxz_vector.begin(), mat_edge3d_maxz_vector.end());
    //     // std::cout << "Vector representing mat_edge_total_vector has " << mat_edge_total_vector.size() << " data points." << std::endl;
    //     // std::cout << "Each vector has " << mat_edge_total_vector[0].size() << " data points." << std::endl;

    //     // for (std::size_t i = 0; i < mat_edge_total_vector.size(); i++)
    //     // {
    //     //     for (std::size_t j = 0; j < mat_edge_total_vector[i].size(); j++)
    //     //     {
    //     //         std::cout << mat_edge_total_vector[i][j] << " , " << std::flush;
    //     //     }
    //     //     std::cout << std::endl;
    //     // }

    //     pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_edge(new pcl::PointCloud<pcl::PointXYZ>);
    //     extracted_edge->width = mat_edge_total_vector.size();
    //     extracted_edge->height = 1;
    //     extracted_edge->points.resize(extracted_edge->width * extracted_edge->height);

    //     // std::cout << "1" << std::endl;

    //     for (std::size_t i = 0; i < extracted_edge->size(); ++i)
    //     {
    //         (*extracted_edge)[i].x = mat_edge_total_vector[i][0];
    //         (*extracted_edge)[i].y = mat_edge_total_vector[i][1];
    //         (*extracted_edge)[i].z = mat_edge_total_vector[i][2];
    //     }

    //     // std::cout << "2" << std::endl;

    //     pcl::PCLPointCloud2* cloud_output4 = new pcl::PCLPointCloud2;
    //     pcl::PCLPointCloud2ConstPtr cloudPtr4(cloud_output4);
    //     pcl::toPCLPointCloud2(*extracted_edge, *cloud_output4);
    //     sensor_msgs::PointCloud2 extracted_edge_ts;
    //     pcl_conversions::fromPCL(*cloud_output4, extracted_edge_ts);
    //     extracted_edge_ts.is_bigendian = false;
    //     extracted_edge_ts.header.seq=1;
    //     extracted_edge_ts.header.stamp=ros::Time::now();
    //     extracted_edge_ts.header.frame_id=cloud_original.header.frame_id;
    //     //segmented_ts.height = cloud_cluster_extracted_ts.height;
    //     //segmented_ts.width = cloud_cluster_extracted_ts.width;
    //     pub4.publish (extracted_edge_ts);
  };
};

///////////////////////////////// main function

int main(int argc, char **argv)
{
  // Initialize ROS
  ROS_INFO("Starting processing node");
  ros::init(argc, argv, "my_pcl_tutorial");

  Subscribe_And_Publish try_object;

  // //Create a ROS subscriber for the input point cloud
  // ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth/color/points", 1, cloud_cb);

  // ROS_INFO("Starting topic /filtered");
  // pub1 = nh.advertise<sensor_msgs::PointCloud2> ("filtered", 1);

  // ROS_INFO("Starting topic /filtered_voxel");
  // pub11 = nh.advertise<sensor_msgs::PointCloud2>("filtered_voxel", 1);

  // ROS_INFO("Starting topic /segmented_ts");
  // pub2 = nh.advertise<sensor_msgs::PointCloud2> ("segmented_ts", 1);

  //   ROS_INFO("Starting topic /segmented_ts_xy");
  //   pub3 = nh.advertise<sensor_msgs::PointCloud2> ("segmented_ts_xy", 1);

  //   ROS_INFO("Starting topic /extracted_edge_ts");
  //   pub4 = nh.advertise<sensor_msgs::PointCloud2> ("extracted_edge_ts", 1);

  // ros::Rate loop_rate(30);

  while (ros::ok())
  {
    ROS_INFO("main 1");
    ros::spinOnce();
    // loop_rate.sleep();
  }
  return 0;
}
