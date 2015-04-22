#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class Callback {
  public:
    void operator()(const PointCloud::ConstPtr& msg)
      {
        ROS_INFO("PointCloud received");
        // pour publier un shared_ptr (mieux)
        PointCloud::Ptr pcl;
        // copie du nuage de point
        *pcl = *msg;
        // TODO : ôter les mauvais points
        publisher.publish(pcl);
        ROS_INFO("PointCloud published");
      }

    Callback(ros::Publisher& pub) : publisher(pub) {}

  private:
    ros::Publisher publisher;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "filtre");
  ros::NodeHandle node;
  ros::Publisher  publisher = node.advertise<PointCloud>("filtre_output", 1);
  Callback callback(publisher);
  ros::Subscriber subscriber = node.subscribe<PointCloud>("filtre_input", 1, callback);
  ROS_INFO("node started");
  ros::spin();
  ROS_INFO("exit");
  return 0;
}
