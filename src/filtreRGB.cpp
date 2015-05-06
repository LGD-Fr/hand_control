#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class Callback {
  public:
    void
    operator()(const PointCloud::ConstPtr& msg)
    {
      PointCloud::Ptr pcl(new PointCloud());
      copy_info(msg, pcl);
      BOOST_FOREACH (const Point& pt, msg->points)
      {
        if (abs(pt.r-red) <= delta and abs(pt.g-green) <= delta and abs(pt.b-blue) <= delta)
          pcl->push_back(pt);
      }
      pcl->height = 1;
      pcl->width = pcl->points.size();
      publisher.publish(pcl);
    }

    Callback(ros::Publisher& pub, uint8_t r, uint8_t g, uint8_t b, uint8_t d)
    : publisher(pub), delta(d), red(r), green(g), blue(b) {}

  private:
    ros::Publisher publisher;
    uint8_t delta;

    uint8_t red;
    uint8_t green;
    uint8_t blue;

    inline
    void
    copy_info(const PointCloud::ConstPtr& a,
              PointCloud::Ptr b)
    {
      b->header = a->header;
      b->sensor_origin_ = a->sensor_origin_;
      b->sensor_orientation_ = a->sensor_orientation_;
      b->is_dense = a->is_dense;
    }
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "filtreRGB");
  ros::NodeHandle node("filtreRGB");

  // récupération des paramètres
  uint8_t red(0);
  uint8_t blue(0);
  uint8_t green(0);
  
  uint8_t delta(0);

  int redInt;
  int greenInt;
  int blueInt;

  int deltaInt;

  if (node.getParam("red",  redInt))
  {
    ROS_INFO("red : %d" , redInt);
  } else {
    node.setParam("red", 0);
    node.getParam("red", redInt);
    ROS_INFO("red : %d (default value)", redInt);
  }

  if (node.getParam("blue", blueInt))
  {
  
    ROS_INFO("blue : %d" , blueInt);
  } else {
    node.setParam("blue", 0);
    node.getParam("blue", blueInt);
    ROS_INFO("blue : %d (default value)", blueInt);
  }

  if (node.getParam("green", greenInt))
  {
    ROS_INFO("green : %d" , greenInt);
  } else {
    node.setParam("green", 0);
    node.getParam("green", greenInt);
    ROS_INFO("green : %d (default value)", greenInt);
  }

  if (node.getParam("delta", deltaInt))
  {
    ROS_INFO("delta : %d" , deltaInt);
  } else {
    node.setParam("delta", 0);
    node.getParam("delta", deltaInt);
    ROS_INFO("delta : %d (default value)", deltaInt);
  }
 

  // initialisatio
  ros::Publisher  publisher = node.advertise<PointCloud>("output", 1);
  Callback callback(publisher, (uint8_t) redInt, (uint8_t) greenInt, (uint8_t) blueInt, (uint8_t) deltaInt);
  ros::Subscriber subscriber = node.subscribe<PointCloud>("input", 1, callback);

  // démarrage
  ROS_INFO("node started");
  ros::spin();
  ROS_INFO("exit");
  return 0;
}
