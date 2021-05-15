#include<iostream>
#include<pcl/io/pcb_io.h>
#include<pcl_conversions/pcl_conversions.h>
#include<pcl/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/kdtree/kdtree_flann.h>
#include<pcl/search/kdtree.h>
#include<pcl/search/impl/kdtree.hpp>

#include<ros/ros.h>
#include<ros/console.h>
#include<sensor_msgs/PointCloud2.h>
#include<geometry_msgs/Vector3.h>
#include<geometry_msgs/PoseStamped.h>
#include<nav_msgs/Odometry.h>
#include<Eigen/Eigen>
#include<math.h>
#include<random>

using namespace std;
using namespace Eigen;

ros::Publisher  _all_map_pub;
ros::Publisher  _all_ground_pub;
ros::Subscriber _odom_sub;

int _obs_num,_cir_num;
double _x_size, _y_size, _z_size, _init_x, _init_y, _resolution, _sense_rate;
double _x_l, _x_h, _y_l, _y_h, _w_l; _w_h, _h_l,_h_h, _w_c_l, _w_c_h;
std::string map_frame_name;

bool _has_odom = false;
bool _ground_map_swt = false;

sensor_msgs::PointCloud2 globalMap_pcd;
sensor_msgs::PointCloud2 globalGround_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

pcl::search::KdTree<pcl::PointXYZ>kdtreeMap;
vector<int> pointIdxSearch;
vector<float> pointSquaredDistance;

void RandomMapGenerate(bool ground_map_swt)
{
    
}

void rcvOdometryCallbck(const nav_msgs::Odometry odom)
{

}

void pubSensedPoints(bool ground_map_swt)
{

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "random_complex_scene");
    ros::NodeHandle n("~");

    _all_map_pub = n.advertise<sensor_msgs::PointCloud2>("global_map",1);
    _all_ground_pub = n.advertise<sensor_msgs::PointCloud2>("global_ground",1);
    _odom_sub = n.subscribe("odometry",50,rcvOdometryCallbck);

    _x_l = -_x_size / 2.0;
    _x_h = +_x_size / 2.0;

    _y_l = -_y_size / 2.0;
    _y_h = +_y_size / 2.0;

    RandomMapGenerate(false);
    RandomMapGenerate(true);
    ros::Rate loop_rate(_sense_rate);
    bool ground_map_swt = true;
    while(ros::ok()){
        pubSensedPoints(ground_map_swt);
        ground_map_swt = !ground_map_swt;
        ros::spinOnce();
        loop_rate.sleep();
    }
}