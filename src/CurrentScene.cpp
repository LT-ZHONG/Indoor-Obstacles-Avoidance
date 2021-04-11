//
// Created by hello on 2021/1/4.
//

#include "obstacles_detection/CurrentScene.h"

CurrentScene::CurrentScene()
{
    cloud_pass.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_rgb.reset(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_filtered.reset(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_smoothed.reset(new pcl::PointCloud<pcl::PointXYZ>);

    ground_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    ground_coe.reset(new pcl::ModelCoefficients);
    ground_indices.reset(new pcl::PointIndices);

    object_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    obstacles_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    cloud_cropBox.reset(new pcl::PointCloud<pcl::PointXYZ>);

    plane_outliers.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void CurrentScene::pass_through(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PassThrough<pcl::PointXYZ> pass ;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, 4.0);
    pass.filter(*cloud_z);

    pass.setInputCloud(cloud_z);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1.0, 1.0);
    pass.filter(*cloud_y);

    pass.setInputCloud(cloud_y);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(-1.5, 1.5);
    pass.filter(*cloud_pass);
}

void CurrentScene::voxel_grid (float leaf_size, int count) const
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> vg ;
    vg.setInputCloud(cloud_pass);
    vg.setLeafSize(leaf_size, leaf_size, leaf_size);
    vg.filter(*cloud_temp);

    pcl::copyPointCloud(*cloud_temp, *cloud_filtered);
    pcl::copyPointCloud(*cloud_temp, *cloud_rgb);

    if (count == 50)
        ROS_INFO_STREAM("Voxel grid -> points.size = " << cloud_filtered->points.size());
}

void CurrentScene::smooth_cloud(float leaf_size) const
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointNormal>::Ptr mls_cloud (new pcl::PointCloud<pcl::PointNormal>);

    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls ;
    mls.setInputCloud(cloud_filtered);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(4 * leaf_size);
    mls.process(*mls_cloud);

    cloud_filtered->clear();
    pcl::copyPointCloud(*mls_cloud, *cloud_filtered);
}

bool CurrentScene::ransac_ground(float leaf_size) const
{
    // limit the cloud below a certain height before performing ransac
    pcl::PointIndices::Ptr indices_tmp (new pcl::PointIndices);

    pcl::PassThrough<pcl::PointXYZ> pass ;
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.50, 1.0);
    pass.filter(indices_tmp->indices);

    // ransac ground
    pcl::SACSegmentation<pcl::PointXYZ> seg ;
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setAxis(Eigen::Vector3f(0.0, 1.0, 0.0));  // Set the axis along which we need to search for a model perpendicular to
    seg.setEpsAngle(pcl::deg2rad(5.0f));  // Set the angle epsilon (delta) threshold
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(4 * leaf_size);  // 10 * 1 cm
    seg.setInputCloud(cloud_filtered);
    seg.setIndices(indices_tmp);
    seg.segment(*ground_indices, *ground_coe);

//    std::cout << "ground_coe = " << ground_coe->values[0] << "  " << ground_coe->values[1] << "  " <<
//                                    ground_coe->values[2] << "  " << ground_coe->values[3] << std::endl ;

    if (ground_indices->indices.size() < 10)
    {
        ROS_WARN_STREAM("Ground -> points.size = " << ground_indices->indices.size());
        return false ;
    }

    pcl::ExtractIndices<pcl::PointXYZ> ex ;
    ex.setInputCloud(cloud_filtered);
    ex.setIndices(ground_indices);
    ex.filter(*ground_cloud);

    ex.setNegative(true);
    ex.filter(*object_cloud);

    return true ;
}

void CurrentScene::extract_obstacles(float threshold, int count) const
{
    pcl::ModelOutlierRemoval<pcl::PointXYZ> mor ;
    mor.setModelCoefficients(*ground_coe);
    mor.setModelType(pcl::SACMODEL_PLANE);
    mor.setInputCloud(object_cloud);
    mor.setNegative(true);
    mor.setThreshold(threshold);  // 10 cm
    mor.filter(*plane_outliers);

    Eigen::Vector4f coe (ground_coe->values[0], ground_coe->values[1],
                         ground_coe->values[2], ground_coe->values[3]);

    pcl::PointIndices::Ptr indices_tmp (new pcl::PointIndices);
    for (int i = 0; i < plane_outliers->points.size(); ++i)
    {
        Eigen::Vector4f point = plane_outliers->points[i].getVector4fMap();

        float dot = coe.dot(point);
        if (dot > 0)
        {
            indices_tmp->indices.push_back(i);
        }
    }

    pcl::ExtractIndices<pcl::PointXYZ> ex ;
    ex.setInputCloud(plane_outliers);
    ex.setIndices(indices_tmp);
    ex.filter(*obstacles_cloud);

    if (count == 50)
        ROS_INFO_STREAM("Obstacles -> points.size = " << obstacles_cloud->points.size());
}

void CurrentScene::extract_near_obstacles() const
{
    pcl::PassThrough<pcl::PointXYZ> pass ;
    pass.setInputCloud(cloud_filtered);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-1.0, 0.50);
    pass.filter(*obstacles_cloud);
}

bool CurrentScene::crop_box(const Eigen::Vector4f& min, const Eigen::Vector4f& max) const
{
    pcl::CropBox<pcl::PointXYZ> cropBox ;
    cropBox.setMin(min);
    cropBox.setMax(max);
    cropBox.setInputCloud(cloud_filtered);
    cropBox.filter(*cloud_cropBox);

    return cloud_cropBox->points.size() > 10 ;
}
