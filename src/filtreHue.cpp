#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
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
          uint8_t min, max, c;
          
          //Recherche du min et du max des coefficients R,G,B
          if (pt.r >= pt.g) {
            if (pt.g >= pt.b) {
              max = pt.r
                min = pt.b;
            } else if (pt.r >= pt.b) {
              max = pt.r;
              min = pt.g;
            } else {
              max = pt.b;
              min = pt.g;
            }
          } else if (pt.r >= pt.b) {
            max = pt.g;
            min = pt.b;
          } else if (pt.g >= pt.b) {
            max = pt.g;
            min = pt.r;
          } else {
            max = pt.b;
            min = pt.r;
          }
          
          c = max - min;
          
          assert(c > 0);
          assert(max > pt.r);
          assert(max > pt.g);
          assert(max > pt.b);
          assert(min < pt.r);
          assert(min < pt.g);
          assert(min < pt.b);
         
          
          
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
