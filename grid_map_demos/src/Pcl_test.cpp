#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_pcl/grid_map_pcl.hpp>
#include <grid_map_pcl/GridMapPclConverter.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <string>
#include <stdlib.h>
#include <iostream>

using namespace grid_map;
GridMapPclConverter *convert;
std::string pathToBag = "grid_map.bag";
std::string topic = "topic";

int main(int argc, char **argv)
{
    ros::init (argc, argv, "Pcl_demo");
    convert = new GridMapPclConverter;
    ros::NodeHandle nh;
    ros::Publisher publisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
    ros::Time time = ros::Time::now ();


    pcl::PolygonMesh Test_mesh;
    GridMap Test_map({"Test_layer"});
    Test_map.setFrameId ("map");

//    pcl::PCLPointCloud2 cloud_blob;
//    pcl::io::loadPCDFile (argv[1], cloud_blob);tutorial_demo
    double time1 = ros::Time::now ().toSec ();
    pcl::io::loadPolygonFileVTK(argv[1], Test_mesh);

    convert->initializeFromPolygonMesh (Test_mesh, 0.01, Test_map);
    convert->addLayerFromPolygonMesh (Test_mesh, "Test_layer",Test_map);
    double time2 = ros::Time::now ().toSec () - time1;

    // publish
    Test_map.setTimestamp (time.toSec ());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage (Test_map,message);
    ros::Rate rate(30.0);
    while (nh.ok())
    {
        publisher.publish (message);
        ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
        rate.sleep();
    }
//    double value;
//    int flag = 0;
//    for (GridMapIterator it(Test_map); !it.isPastEnd(); ++it)
//    {
//        value = Test_map.at("Test_layer", *it);
//        if (value > 0)
//        {
//            value = 1;
//        }
//        flag++;
//    }
//    value = flag;
    GridMapRosConverter::saveToBag(Test_map, pathToBag, topic);
    Test_map.setFrameId ("map");
    return 0;
}