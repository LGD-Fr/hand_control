#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/tracking/impl/hsv_color_coherence.hpp>
#include <assert.h>
#include <dynamic_reconfigure/server.h>
#include <hand_control/FiltreConfig.h>


typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class Callback {
  public:
    void
    callback(const PointCloud::ConstPtr& msg)
    {
      PointCloud::Ptr pcl(new PointCloud());
      copy_info(msg, pcl);
      BOOST_FOREACH (const Point& pt, msg->points)
      {
        if (pt.z < z_max and hue_dist(pt) < delta_hue and sat(pt) < sat_max and sat(pt) > sat_min and val(pt) < val_max and val(pt) > val_min)
          pcl->push_back(pt);
      }
      pcl->height = 1;
      pcl->width = pcl->points.size();
      publisher.publish(pcl);
    }

    Callback(ros::Publisher& pub)
    : publisher(pub), z_max(90.), hue(0.), delta_hue(20.),  sat_min(0.3), sat_max(1.), val_min(0.3), val_max(1.)
    {
      assert(delta_hue > 0);
      assert(z_max > 0);
      assert(hue >= 0);
      assert(hue <= 360.);
      assert(sat_min >= 0);
      assert(sat_max <= 1.);
      assert(val_min >= 0);
      assert(val_max <= 1.);
    }

  void
  reconfigure(hand_control::FiltreConfig& c, uint32_t level) { 
    z_max = c.z_max;
    hue = c.hue;
    delta_hue = c.delta_hue;
    val_min = c.val_min;
    val_max = c.val_max;
    sat_min = c.sat_min;
    sat_max = c.sat_max;
  }

  private:
    ros::Publisher publisher;
    float z_max, hue, delta_hue, val_min, val_max, sat_min, sat_max;

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
      return std::min(diff1, diff2);
    }

    float sat(const Point& pt)
    {
      float h, s, v, diff1, diff2;
      pcl::tracking::RGB2HSV(pt.r, pt.g, pt.b, h, s, v);
      return s;
    }

    float val(const Point& pt)
    {
      float h, s, v, diff1, diff2;
      pcl::tracking::RGB2HSV(pt.r, pt.g, pt.b, h, s, v);
      return v;
    }
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "filtre");
  ros::NodeHandle node("filtre");

  // initialisation
  ros::Publisher  publisher = node.advertise<PointCloud>("output", 1);
  Callback my_callback(publisher);
  ros::Subscriber subscriber = node.subscribe<PointCloud>("input", 1, &Callback::callback, &my_callback);

  dynamic_reconfigure::Server<hand_control::FiltreConfig> server;
  dynamic_reconfigure::Server<hand_control::FiltreConfig>::CallbackType f;
  f = boost::bind(&Callback::reconfigure, &my_callback, _1, _2);
  server.setCallback(f);

  // démarrage
  ROS_INFO("node started");
  ros::spin();
  ROS_INFO("exit");
  return 0;
}
