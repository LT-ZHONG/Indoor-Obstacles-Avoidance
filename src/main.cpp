#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <bondcpp/bond.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "obstacles_detection/Viewer.h"
#include "obstacles_detection/CurrentScene.h"


class App
{
public:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_show ;

    boost::shared_ptr<ros::AsyncSpinner> capture ;

    float voxel_leaf_size = 0.01f ;
    float model_outliers_removal_thresh = 0.05f ;

    Eigen::Vector4f cropBox_min ;
    Eigen::Vector4f cropBox_max ;

    Viewer viewer ;

    bool diagnostic_msg = false ;

    int count = 0 ;
    int count2 = 0 ;

public:
    App () : viewer()
    {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        cloud_show.reset(new pcl::PointCloud<pcl::PointXYZ>);

        cropBox_min << -1.0, -1.0, 0.0, 1.0 ;  // x  y  z  1.0
        cropBox_max << 1.0, 1.0, 0.5, 1.0 ;
    }

    void cloud_callback(const sensor_msgs::PointCloud2::Ptr & cloud_ros) const
    {
        pcl::fromROSMsg(*cloud_ros, *cloud_show);
    }

    void start_from_here()
    {
        ros::NodeHandle nh ;

        ros::Subscriber sub = nh.subscribe("/rs_cloud", 1, &App::cloud_callback, this);  // /camera/depth/color/points

        ros::Publisher pub = nh.advertise<std_msgs::Bool>("/planeObsDetect_diagnostic", 1);

        while (sub.getNumPublishers() == 0)
        {
            sleep(2) ;
        }

        ros::Rate r (10);

        capture.reset(new ros::AsyncSpinner(0));
        capture->start();

        while (nh.ok() and !viewer.cloud_viewer_.wasStopped())
        {
            ++count ;
            if (count == 10)
            {
                std_msgs::Bool msg ;  // 发布心跳报文，以告知此节点还在运行
                msg.data = diagnostic_msg ;
                diagnostic_msg = !diagnostic_msg ;
                pub.publish(msg);

                count = 0 ;
            }

            if (!cloud_show->points.empty())
            {
                ++ count2 ;
                this->execute();
            }

            r.sleep();
        }
        capture->stop();
    }

    void execute()
    {
        viewer.create_axis();

        CurrentScene scene ;

        scene.pass_through(cloud_show);

        scene.voxel_grid(voxel_leaf_size, count2);

//        scene.smooth_cloud(voxel_leaf_size);

        if (scene.crop_box(cropBox_min, cropBox_max))
        {
            scene.extract_near_obstacles();
        }
        else
        {
            if (scene.ransac_ground(voxel_leaf_size))
            {
                scene.extract_obstacles(model_outliers_removal_thresh, count2);  // Model Outlier Removal

//                viewer.show_ground_plane( scene.cloud_pass, scene.ground_coe);
            }
            else
            {
                scene.extract_near_obstacles();
            }
        }

        if (count2 > 50)
            count2 = 0 ;

        viewer.show_cloud(scene.obstacles_cloud, scene.cloud_rgb);
        viewer.show_cropBox(cropBox_min, cropBox_max);

        viewer.cloud_viewer_.spinOnce();

        viewer.cloud_viewer_.removeAllPointClouds();
        viewer.cloud_viewer_.removeAllShapes();
    }
/*
    void octree_tracking(const CurrentScene & scene)
    {
        if (count < octree_window)
        {
            vector_cloud.push_back(scene.object_cloud);
            ++count ;
        }
        else if (!scene.object_cloud->empty())
        {
            ref_object_cloud = vector_cloud.front();

            pcl::octree::OctreePointCloudChangeDetector<pcl::PointXYZ> octree (4 * voxel_leaf_size);
            octree.setInputCloud(ref_object_cloud);
            octree.addPointsFromInputCloud();

            octree.switchBuffers();

            octree.setInputCloud(scene.object_cloud);
            octree.addPointsFromInputCloud();

            std::vector<int> newPointIndices ;

            octree.getPointIndicesFromNewVoxels(newPointIndices);

            for (int newPointIndex : newPointIndices)
            {
                pcl::PointXYZ pt (scene.object_cloud->points[newPointIndex]);
                dynamic_cloud->points.push_back(pt);
            }

            std::cout << "dynamic_cloud->points.size = " << dynamic_cloud->points.size() << std::endl ;

            vector_cloud.erase(vector_cloud.begin());
            vector_cloud.push_back(scene.object_cloud);
        }
    }*/
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plane_obstacles_detection");

    App app ;
    app.start_from_here();

    return 0 ;
}