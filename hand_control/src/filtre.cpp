#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Callback {
  public:
    void operator()(const PointCloud::ConstPtr& msg)
      {
        // copie du nuage de point
        PointCloud pcl = *msg;
        // TODO : ôter les mauvais points
        publisher.publish(pcl);
      }
    Callback(ros::NodeHandle& node)
      {
        publisher = node.advertise<PointCloud>("output", 1);
      }
  private:
    ros::Publisher publisher;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filtre");
  ros::NodeHandle node;
  Callback callback(node);
  ros::Subscriber subscriber = node.subscribe<PointCloud>("input", 1, callback);
  ros::spin();
  return 0;
}
