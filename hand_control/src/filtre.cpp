#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

class Filtre {
  private:
    typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
    ros::NodeHandle node;
    ros::Subscriber sub;
    void callback(PointCloud&);
  public:
    Filtre()
    {
      ros::init(argc, argv, "filtre");
      callback(PointCloud& msg)
      {
        
      }
      subscriber = node.subscribe<PointCloud>("input", 1, callback);
      ros::spin();
    }
};

int main(int argc, char **argv)
{
  Filtre f();
}
