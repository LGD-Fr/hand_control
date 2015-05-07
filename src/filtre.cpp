#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/tracking/impl/hsv_color_coherence.hpp>
#include <assert.h>


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
        if (pt.z < zmax and hue_dist(pt) < delta_hue)
          pcl->push_back(pt);
      }
      pcl->height = 1;
      pcl->width = pcl->points.size();
      publisher.publish(pcl);
    }

    Callback(ros::Publisher& pub, float z, float h, float delta_h)
    : publisher(pub), zmax(z), hue(h), delta_hue(delta_h)
    {
      assert(delta_hue > 0);
      assert(zmax > 0);
      assert(hue >= 0);
      assert(hue <= 360.);
    }

  private:
    ros::Publisher publisher;
    float zmax, hue, delta_hue;

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

    inline
    float
    hue_dist(const Point& pt)
    {
      float h, s, v, diff1, diff2;
      pcl::tracking::RGB2HSV(pt.r, pt.g, pt.b, h, s, v);
      h *= 360.0f ;
      diff1 = std::fabs(h - hue);
      if (h < hue)
        diff2 = std::fabs(360.0f + h - hue);
      else
        diff2 = std::fabs(360.0f + hue - h);
      return std::max(diff1, diff2);
    }
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

  double hue(0);
  if (node.getParam("hue", hue))
  {
    ROS_INFO("hue : %f" , hue);
  } else {
    node.setParam("hue", 0.0);
    node.getParam("hue", hue);
    ROS_INFO("hue : %f (default value)", hue);
  }

  double delta_hue(0);
  if (node.getParam("delta_hue", delta_hue))
  {
    ROS_INFO("delta_hue : %f" , delta_hue);
  } else {
    node.setParam("delta_hue", 10.0);
    node.getParam("delta_hue", delta_hue);
    ROS_INFO("delta_hue : %f (default value)", delta_hue);
  }

  // initialisatio
  ros::Publisher  publisher = node.advertise<PointCloud>("output", 1);
  Callback callback(publisher, (float) zmax, (float) hue, (float) delta_hue);
  ros::Subscriber subscriber = node.subscribe<PointCloud>("input", 1, callback);

  // démarrage
  ROS_INFO("node started");
  ros::spin();
  ROS_INFO("exit");
  return 0;
}
