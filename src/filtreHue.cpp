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
        int h(0);
	float rScaled(pt.r/255);
	float gScaled(pt.g/255);
	float bScaled(pt.b/255);

        float cMax(std::max(std::max(rScaled, gScaled), bScaled));
	float cMin(std::min(std::min(rScaled, gScaled), bScaled));
        float cDelta(cMin-cMax);

	if (cMax == rScaled){
	  h = (int) 60*(gScaled-bScaled)/cDelta;
	}else if (cMax = gScaled){
	  h = (int) 60*(2+(bScaled-rScaled)/cDelta);
	}else if (cMax = rScaled){
	  h = (int) 60*(4+(rScaled-gScaled)/cDelta);
        }

	if (h < 0) {
	  h += 360;
	}

	if (abs(h - hue) < delta) {
	  pcl->push_back(pt);
	}
      }
      pcl->height = 1;
      pcl->width = pcl->points.size();
      publisher.publish(pcl);
    }

    Callback(ros::Publisher& pub, int h,  int d)
    : publisher(pub), delta(d), hue(h)  {}

  private:
    ros::Publisher publisher;
    int delta;

    int hue;

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
  ros::init(argc, argv, "filtreHue");
  ros::NodeHandle node("filtreHue");

  // récupération des paramètres
  int hue(0);  
  int delta(0);

  if (node.getParam("hue", hue))
  {
    ROS_INFO("hue : %d" , hue);
  } else {
    node.setParam("hue", 0);
    node.getParam("hue", hue);
    ROS_INFO("hue : %d (default value)", hue);
  }

  if (node.getParam("delta", delta))
  {
    ROS_INFO("delta : %d" , delta);
  } else {
    node.setParam("delta", 0);
    node.getParam("delta", delta);
    ROS_INFO("delta : %d (default value)", delta);
  }
 

  // initialisatio
  ros::Publisher  publisher = node.advertise<PointCloud>("output", 1);
  Callback callback(publisher, (int) hue, (int) delta);
  ros::Subscriber subscriber = node.subscribe<PointCloud>("input", 1, callback);

  // démarrage
  ROS_INFO("node started");
  ros::spin();
  ROS_INFO("exit");
  return 0;
}
