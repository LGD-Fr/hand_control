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

      float x, y, z, h, c;
      x = y = z = h = c = 0.;

      // indices : tout le PointCloud
      std::vector<int> indices;
      for (int i = 0; i < msg->points.size(); ++i)
        indices.push_back(i);

      // vérifier signature
      estimator.computePointNormal(*msg, indices, x, y, z, c);
      h = altitude(msg);

      // publication
      ROS_INFO("Plan published");
      publisher.publish(to_Plan(x, y, z, h, c));
    }

    Callback(ros::Publisher& pub) : publisher(pub), estimator() {}

  private:
    ros::Publisher publisher;
    pcl::NormalEstimationOMP<Point, pcl::Normal> estimator;

    inline
    const hand_control::Plan::ConstPtr
    to_Plan(const float& x, const float& y,
            const float& z, const float& h,
            const float& c)
    {
      hand_control::Plan::Ptr ros_msg(new hand_control::Plan());
      ros_msg->normal.x = x; 
      ros_msg->normal.y = y; 
      ros_msg->normal.z = z; 
      ros_msg->altitude = h;
      ros_msg->curvature = c; 
      return ros_msg;
    }

    inline
    float
    altitude(const PointCloud::ConstPtr& pcl)
    {
      int s = pcl->points.size();
      float h(0);
      for (int i = 0; i < s; ++i)
        h += pcl->points[i].z;
      return h/( (float) s );
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
