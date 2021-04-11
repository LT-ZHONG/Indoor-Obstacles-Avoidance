//
// Created by hello on 2021/1/4.
//

#ifndef SRC_CURRENTSCENE_H
#define SRC_CURRENTSCENE_H

#include <ros/console.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/io.h>
#include <pcl/common/angles.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/model_outlier_removal.h>

#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/surface/mls.h>

#include <pcl/search/kdtree.h>

class CurrentScene
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pass ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_rgb ;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropBox ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_smoothed ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered ;

    pcl::PointCloud<pcl::PointXYZ>::Ptr ground_cloud ;
    pcl::ModelCoefficients::Ptr ground_coe ;
    pcl::PointIndices::Ptr ground_indices ;

    pcl::PointCloud<pcl::PointXYZ>::Ptr object_cloud ;

    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacles_cloud ;

    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_outliers ;

public:
    CurrentScene();

    void pass_through(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud) const;

    void voxel_grid(float leaf_size, int count) const;

    void smooth_cloud(float leaf_size) const;

    bool ransac_ground(float leaf_size) const;

    void extract_obstacles(float threshold, int count) const;

    void extract_near_obstacles() const;

    bool crop_box(const Eigen::Vector4f& min, const Eigen::Vector4f& max) const;
};


#endif //SRC_CURRENTSCENE_H
