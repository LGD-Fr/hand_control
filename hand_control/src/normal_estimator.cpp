#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <geometry_msgs/Quaternion.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef Eigen::Vector4f PCLCoord; // vecteur, accès par v(0), v(1)...
typedef geometry_msgs::Quaternion ROSCoord; // struct : x, y, z, w

class Callback {
  public:
    void
    operator()(const PointCloud::ConstPtr& msg)
    {
      ROS_INFO("PointCloud received");
      PCLCoord pcl_coord();
      float curvature;
      std::vector<int> indices();
      // TODO : choisir tous les indices
      estimator.computePointNormal(*msg, indices,
                                    PCLCoord, curvature);
      /* TODO
      if (curvature <  ?
        publisher.publish(to_ROSCoord(pcl_coord));
        */
    }

    Callback(ros::Publisher& pub) : publisher(pub), estimator() {}

  private:
    ros::Publisher publisher;

    pcl::NormalEstimationOMP<Point, pcl::Normal> estimator;

    const ROSCoord::ConstPtr
    to_ROSCoord(const PCLCoord& pcl_coord)
    {
      ROSCoord::Ptr ros_coord(new ROSCoord());
      ros_coord->x = pcl_coord(0); // a
      ros_coord->y = pcl_coord(1); // b
      ros_coord->z = pcl_coord(2); // c
      ros_coord->w = pcl_coord(3); // d
      return ros_coord;
    }
};

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
