#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class Callback {
  public:
    void
    operator()(const PointCloud::ConstPtr& msg)
    {
      PointCloud::Ptr pcl(new PointCloud());
      for (int i = 0; i < msg->points.size(); ++i)
      {
        if (msg->points[i].z < zmax)
          pcl->points.push_back(msg->points[i]);
      }
      publisher.publish(pcl);
    }

    Callback(ros::Publisher& pub, float z)
    : publisher(pub), zmax(z) {}

  private:
    ros::Publisher publisher;
    float zmax;
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "filtre");
  ros::NodeHandle node("filtre");

  // récupération des paramètres
  double zmax(0);
  if (node.getParam("zmax", zmax))
  {
    ROS_INFO("zmax : %f" , zmax);
  } else {
    node.setParam("zmax", 50.0);
    node.getParam("zmax", zmax);
    ROS_INFO("zmax : %f (default value)", zmax);
  }

  // initialisatio
  ros::Publisher  publisher = node.advertise<PointCloud>("output", 1);
  Callback callback(publisher, (float) zmax);
  ros::Subscriber subscriber = node.subscribe<PointCloud>("input", 1, callback);

  // démarrage
  ROS_INFO("node started");
  ros::spin();
  ROS_INFO("exit");
  return 0;
}
