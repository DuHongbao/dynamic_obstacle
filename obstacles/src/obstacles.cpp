#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  uint32_t shape = visualization_msgs::Marker::SPHERE;



    int obs_pose = 0;
  while (ros::ok())
  {
    visualization_msgs::Marker marker,marker1;




    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "shapes1";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    marker1.header.frame_id = "map";
    marker1.header.stamp = ros::Time::now();
    marker1.ns = "shapes2";
    marker1.id = 1;
    marker1.type = shape;
    marker1.action = visualization_msgs::Marker::ADD;



    marker.pose.position.x = obs_pose/5.0;
    marker.pose.position.y = obs_pose/-5.0;
    marker.pose.position.z = obs_pose/10.0+2;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = obs_pose/10.0;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 0.5;

    marker.lifetime = ros::Duration();


    marker1.pose.position.x = -obs_pose/5.0+1.5;
    marker1.pose.position.y = obs_pose/-5.0;
    marker1.pose.position.z = obs_pose/10.0+1;
    marker1.pose.orientation.x = obs_pose/10.0;
    marker1.pose.orientation.y = 0.0;
    marker1.pose.orientation.z = 0.0;
    marker1.pose.orientation.w = 1.0;

    marker1.scale.x = 1.0;
    marker1.scale.y = 1.0;
    marker1.scale.z = 1.5;
    marker1.color.r = 1.0f;
    marker1.color.g = 0.0f;
    marker1.color.b = 0.0f;
    marker1.color.a = 0.5;

    marker1.lifetime = ros::Duration();



    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }

      sleep(1);
    }
    obs_pose++;    
    if(obs_pose>15){
        obs_pose = -15;
    }


    marker_pub.publish(marker);
    marker_pub.publish(marker1);
    r.sleep();
  }
}