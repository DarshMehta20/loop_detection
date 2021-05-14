// Header
#include "edge_extraction.hpp"

//sorting function
bool sortcol_xy(const std::vector<double> &v1, const std::vector<double> &v2)
{
    return v1[3] < v2[3];
}

edge_extract::edge_extract()
{

    nh.getParam("/initial_radius", initialradius);//radius for kdtree search to bifurcate extreme edge points from internal cluster points and internal plane points
    nh.getParam("/cluster_tree_radius", clusterradius);//radius for kdtree search to remove noise points in cluster extraction

    nh.getParam("/voxel_grid_size", voxel_grid_size);//initial voxel grid size

    //parameter to sort internal points froms cluster points
    nh.getParam("/external_edge_neigh_points", ext_points);
    nh.getParam("/internal_edge_neigh_points", int_points);

    sub_yas_tcp = nh.subscribe<geometry_msgs::Point>("/yas_tcp_camera", 1, &edge_extract::tcp_yas_cb, this);

    sub_abb_tcp = nh.subscribe<geometry_msgs::Point>("/abb_tcp_camera", 1, &edge_extract::tcp_abb_cb, this);

    sub = nh.subscribe<sensor_msgs::PointCloud2>("camera/depth/color/points", 1, &edge_extract::cloud_cb, this);

    ROS_INFO("Starting topic /filtered_voxel");
    pub1 = nh.advertise<sensor_msgs::PointCloud2>("filtered_voxel", 1000);//extracted tshirt publisher

    ROS_INFO("Starting topic /extracted_edge_ts");
    pub2 = nh.advertise<sensor_msgs::PointCloud2>("extracted_edge_ts", 1000);//extracted edge publisher
}

//tcp from yaskawa
void edge_extract::tcp_yas_cb(const geometry_msgs::Point input_yas)
{
    x_yas = input_yas.x;
    y_yas = input_yas.y;
    z_yas = input_yas.z;
}

//tcp from abb
void edge_extract::tcp_abb_cb(const geometry_msgs::Point input_abb)
{
    x_abb = input_abb.x;
    y_abb = input_abb.y;
    z_abb = input_abb.z;
}

void edge_extract::cloud_cb(const sensor_msgs::PointCloud2ConstPtr &input)
{
    //timer start
    pcl::console::TicToc pcl_time;
    pcl_time.tic();

    //x limits
    xmin = x_yas - 0.01;
    xmax = x_abb + 0.01;

    //y limits
    if (y_yas <= y_abb)
    {
        ymin = y_yas - 0.05;
        ymax = y_abb + 0.25;
    }
    else
    {
        ymin = y_abb - 0.05;
        ymax = y_yas + 0.25;
    }

    //z limit
    if (z_yas <= z_abb)
    {
        zmin = z_yas - 0.15;
        zmax = z_abb + 0.1;
    }
    else
    {
        zmin = z_abb - 0.15;
        zmax = z_yas + 0.1;
    }

    if (zmin < 0.52)
    {
        zmin = 0.52;
    }

    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud_original;
    pcl::fromROSMsg(*input, cloud_original);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    //tshirt extraxtion
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud_original.makeShared());
    pass.setFilterFieldName("z");
    pass.setFilterLimits(zmin, zmax);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(xmin, xmax);
    pass.filter(*cloud_filtered);
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(ymin, ymax);
    pass.filter(*cloud_filtered);

    //Voxelization
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_filtered);
    vg.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
    vg.filter(*cloud_filtered);

    //publish filtered voxel data
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
    pub1.publish(filtered_voxel);

    //timer stop
    time_pcl = pcl_time.toc() / 1000;
    std::cout << "Time taken till filtered voxel " << time_pcl << " s" << std::endl;


    pcl_time.tic();    
    //generating a secondary 2d pt-cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZ>);

    *cloud = *cloud_filtered;
    *cloud_xy = *cloud;

    for (std::size_t i = 0; i < cloud_xy->size(); ++i)
    {
        (*cloud_xy)[i].z = 0;
    }
    time_pcl = pcl_time.toc() / 1000;
    std::cout << "Time taken after filtered voxel till first kdtree is " << time_pcl << " s" << std::endl;

    pcl_time.tic();
    //kd tree search for 2d pt-cl
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(cloud_xy);
    pcl::PointXYZ searchPoint;

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    float radius = initialradius; //search radius
    std::vector<std::vector<int>> I_2d;
    std::vector<int> ncols_2d;
    //index vectors for internal and external points
    std::vector<int> int_1_idx;  // useless pts
    std::vector<int> edge_1_idx; //outer edge
    std::vector<int> edge_2_idx; //inner cluster

    for (std::size_t i = 0; i < cloud_xy->size(); ++i)
    {
        searchPoint.x = (*cloud_xy)[i].x;
        searchPoint.y = (*cloud_xy)[i].y;
        searchPoint.z = (*cloud_xy)[i].z;
        if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0)
        {
            I_2d.push_back(pointIdxRadiusSearch);            //index of point inside the radius
            ncols_2d.push_back(pointIdxRadiusSearch.size()); //number of points in the radius
            if (pointIdxRadiusSearch.size() < ext_points)
            {
                edge_1_idx.push_back(i); //external edges
            }
            else if (pointIdxRadiusSearch.size() > int_points)
            {
                edge_2_idx.push_back(i); //internal clusters
            }
            else
            {
                int_1_idx.push_back(i); //internal flat surfaces which are ignored
            }
        }
    }
    time_pcl = pcl_time.toc() / 1000;
    std::cout << "Time taken after first kdtree is " << time_pcl << " s" << std::endl;

    /////////////////////////////////////////////////////////
    pcl_time.tic();
    //creating a 4th column to sort data and grouping x values
    std::vector<std::vector<double>> mat_top3d_noNAN_vector;
    for (std::size_t i = 0; i < cloud->size(); ++i)
    {
        std::vector<double> *store_point = new std::vector<double>(4);
        (*store_point)[0] = (*cloud)[i].x;// Taking the entire top edge without the use of any edge detecting kdtree search thus cloud
        (*store_point)[1] = (*cloud)[i].y;
        (*store_point)[2] = (*cloud)[i].z;
        (*store_point)[3] = 0;
        mat_top3d_noNAN_vector.push_back(*store_point);
        mat_top3d_noNAN_vector[i][0] = (floor(mat_top3d_noNAN_vector[i][0] / 0.005) + 1) * 0.005;
        mat_top3d_noNAN_vector[i][3] = 1000 * mat_top3d_noNAN_vector[i][0] - mat_top3d_noNAN_vector[i][1];
        delete store_point;
    }
    time_pcl = pcl_time.toc() / 1000;
    std::cout << "Time taken for floor is " << time_pcl << " s" << std::endl;

    pcl_time.tic();
    sort(mat_top3d_noNAN_vector.begin(), mat_top3d_noNAN_vector.end(), sortcol_xy);//sorting 4th column created earlier
    time_pcl = pcl_time.toc() / 1000;
    std::cout << "Time taken for sort 1 is " << time_pcl << " s" << std::endl;

    pcl_time.tic();
    //extracting top edge
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
    time_pcl = pcl_time.toc() / 1000;
    std::cout << "Time taken after first kdtree till top edge is " << time_pcl << " s" << std::endl;

//bottom cluster loop extraction
    pcl_time.tic();
    //internal clusters edge removal
    pcl::PointCloud<pcl::PointXYZ>::Ptr mat_edge3d_noNAN(new pcl::PointCloud<pcl::PointXYZ>);
    for (std::size_t i = 0; i < edge_2_idx.size(); i++)
    {
        mat_edge3d_noNAN->push_back((*cloud)[edge_2_idx[i]]);
    }
    mat_edge3d_noNAN->width = mat_edge3d_noNAN->size();
    mat_edge3d_noNAN->height = 1;
    mat_edge3d_noNAN->is_dense = true;
//making a 2d pt-cloud just like before
    pcl::PointCloud<pcl::PointXYZ>::Ptr mat_edge2d_noNAN(new pcl::PointCloud<pcl::PointXYZ>);
    *mat_edge2d_noNAN = *mat_edge3d_noNAN;
    for (std::size_t i = 0; i < mat_edge2d_noNAN->size(); ++i)
    {
        (*mat_edge2d_noNAN)[i].z = 0;
    }
    time_pcl = pcl_time.toc() / 1000;
    std::cout << "Time taken after top edge till second kdtree is " << time_pcl << " s" << std::endl;


    pcl_time.tic();
    //kdtree search to find and remove noise points using neighbour quatity filtering
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree_edge2d;
    kdtree_edge2d.setInputCloud(mat_edge2d_noNAN);
    pcl::PointXYZ searchPoint_edge2d;

    std::vector<int> pointIdxRadiusSearch_edge2d;
    std::vector<float> pointRadiusSquaredDistance_edge2d;

    float radius_edge2d = clusterradius;
    std::vector<std::vector<int>> I_edge2d;
    std::vector<int> ncols_edge2d;
    std::vector<int> edge_removed_noise_point;
    std::vector<std::vector<double>> mat_edge3d_vector;

//kdtree serach while creating 4th column and grouping x values
    for (std::size_t i = 0; i < mat_edge2d_noNAN->size(); ++i)
    {
        searchPoint_edge2d.x = (*mat_edge2d_noNAN)[i].x;
        searchPoint_edge2d.y = (*mat_edge2d_noNAN)[i].y;
        searchPoint_edge2d.z = (*mat_edge2d_noNAN)[i].z;

        if (kdtree_edge2d.radiusSearch(searchPoint_edge2d, radius, pointIdxRadiusSearch_edge2d, pointRadiusSquaredDistance_edge2d) > 0)
        {
            I_edge2d.push_back(pointIdxRadiusSearch_edge2d);
            ncols_edge2d.push_back(pointIdxRadiusSearch_edge2d.size());
            if (pointIdxRadiusSearch_edge2d.size() > 3)
            {
                edge_removed_noise_point.push_back(i);
                std::vector<double> *store_point = new std::vector<double>(4);
                (*store_point)[0] = (*mat_edge3d_noNAN)[i].x;
                (*store_point)[1] = (*mat_edge3d_noNAN)[i].y;
                (*store_point)[2] = (*mat_edge3d_noNAN)[i].z;
                (*store_point)[3] = 0;
                mat_edge3d_vector.push_back(*store_point);
                mat_edge3d_vector[mat_edge3d_vector.size() - 1][0] = (floor(mat_edge3d_vector[mat_edge3d_vector.size() - 1][0] / 0.005) + 1) * 0.005;
                mat_edge3d_vector[mat_edge3d_vector.size() - 1][3] = 1000 * mat_edge3d_vector[mat_edge3d_vector.size() - 1][0] + mat_edge3d_vector[mat_edge3d_vector.size() - 1][1];
                delete store_point;
            }
        }
    }
    time_pcl = pcl_time.toc() / 1000;
    std::cout << "Time taken for second kdtree is " << time_pcl << " s" << std::endl;
    
    pcl_time.tic();
    sort(mat_edge3d_vector.begin(), mat_edge3d_vector.end(), sortcol_xy);
    time_pcl = pcl_time.toc() / 1000;
    std::cout << "Time taken for sort 2 is " << time_pcl << " s" << std::endl;

    pcl_time.tic();
    //topmost cluster extarction
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
    
    //maximum z in topmost cluster extraction
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
    time_pcl = pcl_time.toc() / 1000;
    std::cout << "Time taken after second kdtree till bottom loop extraction is " << time_pcl << " s" << std::endl;

    pcl_time.tic();
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

    pcl::PCLPointCloud2 *cloud_output4 = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr4(cloud_output4);
    pcl::toPCLPointCloud2(*extracted_edge, *cloud_output4);
    sensor_msgs::PointCloud2 extracted_edge_ts;
    pcl_conversions::fromPCL(*cloud_output4, extracted_edge_ts);
    extracted_edge_ts.is_bigendian = false;
    extracted_edge_ts.header.seq = 1;
    extracted_edge_ts.header.stamp = ros::Time::now();
    extracted_edge_ts.header.frame_id = cloud_original.header.frame_id;
    pub2.publish(extracted_edge_ts);

    //timer stop
    time_pcl = pcl_time.toc() / 1000;
    std::cout << "Time taken for rest is " << time_pcl << " s" << std::endl;

    std::cout << "-----------------------------------" << std::endl;
}

//////////////////////////////////////////////////////////////////////
//Main function
/////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
    // Initialize ROS
    ROS_INFO("Starting processing node");
    ros::init(argc, argv, "loop_detection_original");

    edge_extract extract_object;

    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}
