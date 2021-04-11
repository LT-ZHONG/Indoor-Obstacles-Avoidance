//
// Created by hello on 2021/1/4.
//

#include "obstacles_detection/Viewer.h"

Viewer::Viewer()
{
    cloud_viewer_.setPosition (0, 0);
    cloud_viewer_.setSize (1600, 800);
    cloud_viewer_.setCameraClipDistances (0.0, 10.0);
    cloud_viewer_.setCameraPosition(0, 0, 0, 0, 0, 1, 0, -1, 0);
    cloud_viewer_.setCameraFieldOfView(1);

    v1 = 0 ;
    v2 = 0 ;
}

void Viewer::create_axis()
{
    axis.reset(new pcl::PointCloud<pcl::PointXYZ>);

    axis->points.emplace_back(0, 0, 0);
    axis->points.emplace_back(0.5, 0, 0);
    axis->points.emplace_back(0, 0.5, 0);
    axis->points.emplace_back(0, 0, 0.5);

    cloud_viewer_.addArrow(axis->points[1], axis->points[0], 1.0, 0.0, 0.0, false, "x") ;
    cloud_viewer_.addArrow(axis->points[2], axis->points[0], 0.0, 1.0, 0.0, false, "y") ;
    cloud_viewer_.addArrow(axis->points[3], axis->points[0], 0.0, 0.0, 1.0, false, "z") ;
}

void Viewer::show_ground_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_rgb, pcl::ModelCoefficients::Ptr& coefficients)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_rgb);
    cloud_viewer_.addPointCloud<pcl::PointXYZRGB> (cloud_rgb, rgb, "cloud_rgb");

    cloud_viewer_.addPlane(*coefficients, "plane");
}

void Viewer::show_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_rgb)
{

    cloud_viewer_.addPointCloud<pcl::PointXYZ> (cloud_rgb, "cloud_rgb");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(cloud, 255, 0, 0);
    cloud_viewer_.addPointCloud<pcl::PointXYZ>(cloud, red, "cloud");
    cloud_viewer_.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");

}

void Viewer::show_cropBox(Eigen::Vector4f min, Eigen::Vector4f max)
{
    float radius = 0.02f ;

    pcl::PointXYZ min_point(min[0], min[1], min[2]);
    pcl::PointXYZ max_point(max[0], max[1], max[2]);

    cloud_viewer_.addSphere(min_point, radius, 255.0, 0.0, 0.0, "min");
    cloud_viewer_.addSphere(max_point, radius, 0.0, 255.0, 0.0, "max");

    cloud_viewer_.addCube(min_point.x, max_point.x, min_point.y, max_point.y, min_point.z, max_point.z, 1.0, 1.0, 1.0, "cube");
}