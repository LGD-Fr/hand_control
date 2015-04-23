#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <string>
#include <sstream>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class Callback {
  public:
    void
      operator()(const PointCloud::ConstPtr& msg)
      {
        std::ostringstream stream;
        stream << "PointCloud published :" << std::endl;
        for(int i = 0; i < msg->points.size(); ++i)
        {
          pcl::PointXYZRGB p = msg->points[i];
          stream << std::endl
            << "point # " << i << std::endl
            << "x : " << p.x << std::endl
            << "y : " << p.y << std::endl
            << "z : " << p.z << std::endl
            << "r : " << (int) p.r << std::endl
            << "g : " << (int) p.g << std::endl
            << "b : " << (int) p.b << std::endl;
        }
          ROS_INFO("%s", stream.str().c_str());
      }
};

  int
main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_displayer");
  ros::NodeHandle node("pcl_displayer");

  // initialisation
  ros::Subscriber subscriber = node.subscribe<PointCloud>("input", 1, Callback());

  // démarrage
  ROS_INFO("node started");
  ros::spin();
  ROS_INFO("exit");
  return 0;
}
