#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Eigen>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <vector>
class  OBSTACLE {  
  private:
    //Subscribers
    ros::Subscriber  _pcl_sub;
    ros::Subscriber _odom_sub;
    //Publishers
    ros::Publisher  _modMatrix_pub;       
    ros::Publisher _vis_obs;
    ros::Publisher _vis_traj_points;
    // obstacles configuration
    int obs_id;           
    Eigen::Vector3d coord;      //position of obstacle
    Eigen::Matrix3d Rot;                         //rotation matrix of obstacle
    Eigen::Matrix3d modMatrix, matrixE, matrixD;
    double  obs_a,obs_b,obs_c;    //elipsoid  parameter
    double roll,pitch,yaw;          //obstacle pose
    pcl::PointCloud<pcl::PointXYZ> obsCloud;    //moving obstacle
    visualization_msgs::Marker obsMarker;


  public:
    OBSTACLE();
    ~OBSTACLE();
    void obstacle_reshape();  
    Eigen::Matrix3d getE();
    Eigen::Matrix3d getD();
    Eigen::Matrix3d getMod();


    void rcvOdometryCallback(const nav_msgs::Odometry & odom);
    void rcvObsPointCloudCallBack(const senser_msgs::PointCloud2 &moving_pointcloud);

    void pubModMatrix();
    void pubObsVis();
    
};
