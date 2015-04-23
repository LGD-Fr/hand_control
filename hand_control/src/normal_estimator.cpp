#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <geometry_msgs/Quaternion.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef Eigen::Vector4f PCLCoord; // vecteur, accès par v(0), v(1)...
typedef geometry_msgs::Quaternion ROSCoord; // struct : x, y, z, w

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "normal_estimator");
  ros::NodeHandle node("estimator");

  // initialisation
  ros::Publisher  publisher = node.advertise<ROSCoord>("output", 1);
  Callback callback(publisher);
  ros::Subscriber subscriber = node.subscribe<PointCloud>("input", 1, callback);

  // démarrage
  ROS_INFO("node started");
  ros::spin();
  ROS_INFO("exit");
  return 0;
}
