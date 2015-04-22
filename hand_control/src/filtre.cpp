#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class Callback
{
  public:
    operator()(const PointCloud::Ptr& msg)
    {
      pub.publish(*msg);
    }

    Callback(ros::NodeHandle& node)
    {
      pub = node.advertise<PointCloud>("output", 1);
    }

  private:
    ros::Publisher pub;
};

int
main(int argc, char **argv)
{
  ros::init(argc, argv, "filtre");
  ros::NodeHandle node;
  Callback callback(node);
  ros::Subscriber subscriber = node.subscribe<PointCloud>("input", 1, callback);
  ros::spin();
  return 0;
}
