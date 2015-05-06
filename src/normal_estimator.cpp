#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <hand_control/Plan.h>
#include <time.h>

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
      publisher.publish(to_Plan(x, y, z, h, c, msg->header.seq, msg->header.stamp));
    }

    Callback(ros::Publisher& pub) :
      publisher(pub), estimator() {}

  private:
    ros::Publisher publisher;
    pcl::NormalEstimationOMP<Point, pcl::Normal> estimator;

    inline
    const hand_control::Plan::ConstPtr
    to_Plan(const float& x, const float& y,
            const float& z, const float& h,
            const float& c, uint32_t seq, uint64_t msec64)
    {
      hand_control::Plan::Ptr ros_msg(new hand_control::Plan());
      ros_msg->normal.x = x; 
      ros_msg->normal.y = y; 
      ros_msg->normal.z = z; 
      ros_msg->altitude = h;
      ros_msg->curvature = c; 
      // uint64_t msec64 is in ms (10-6)
      uint64_t sec64 = msec64 / 1000000;
      uint64_t nsec64 = (msec64 % 1000000) * 1000;
      ros_msg->header.stamp.sec = (uint32_t) sec64;
      ros_msg->header.stamp.nsec = (uint32_t) nsec64;
      ros_msg->header.seq = seq;
      ros_msg->header.frame_id = "0";
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
