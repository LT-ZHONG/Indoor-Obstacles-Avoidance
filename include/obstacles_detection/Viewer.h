//
// Created by hello on 2021/1/4.
//

#ifndef SRC_VIEWER_H
#define SRC_VIEWER_H

#include <pcl/visualization/pcl_visualizer.h>

class Viewer
{
public:
    pcl::visualization::PCLVisualizer cloud_viewer_ ;

    pcl::PointCloud<pcl::PointXYZ>::Ptr axis ;

    int v1, v2 ;

public:
    Viewer();

    void create_axis();

    void show_ground_plane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud_rgb, pcl::ModelCoefficients::Ptr& coefficients);

    void show_cloud (pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud_rgb);

    void show_cropBox(Eigen::Vector4f min, Eigen::Vector4f max);
};


#endif //SRC_VIEWER_H
