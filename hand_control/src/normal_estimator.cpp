#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <hand_control/Plan.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class Callback {
  public:
    void
    operator()(const PointCloud::ConstPtr& msg)
    {
      ROS_INFO("PointCloud received");
      Eigen::Vector4f pcl_coord;
      float curvature;
      std::vector<int> indices;
      // indices : tout le PointCloud
      for (int i = 0; i < msg->points.size(); ++i)
        indices.push_back(i);
      estimator.computePointNormal(*msg, indices,
                                    pcl_coord, curvature);
      // publication
      ROS_INFO("Plan published");
      publisher.publish(to_Plan(pcl_coord, curvature));
    }

    Callback(ros::Publisher& pub) : publisher(pub), estimator() {}

  private:
    ros::Publisher publisher;

    //pcl::NormalEstimationOMP<Point, Eigen::Vector4f> estimator;
    pcl::NormalEstimationOMP<Point, pcl::Normal> estimator;

    const hand_control::Plan::ConstPtr
    to_Plan(const Eigen::Vector4f& pcl_coord, const float& curvature)
    {
      hand_control::Plan::Ptr ros_msg(new hand_control::Plan());
      ros_msg->normal.x = pcl_coord(0); // a
      ros_msg->normal.y = pcl_coord(1); // b
      ros_msg->normal.z = pcl_coord(2); // c
      ros_msg->altitude = pcl_coord(3); // d
      ros_msg->curvature = curvature; // \lambda
      return ros_msg;
    }
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "normal_estimator");
  ros::NodeHandle node("estimator");

  // initialisation
  ros::Publisher  publisher = node.advertise<hand_control::Plan>("output", 1);
  Callback callback(publisher);
  ros::Subscriber subscriber = node.subscribe<PointCloud>("input", 1, callback);

  // démarrage
  ROS_INFO("node started");
  ros::spin();
  ROS_INFO("exit");
  return 0;
}
