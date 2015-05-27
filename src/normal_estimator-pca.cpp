#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <hand_control/Plan.h>
#include <time.h>
#include <math.h>

#include <pcl/common/pca.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef Eigen::Matrix3f& Matrix;

class Callback {
  public:
    void
    operator()(const PointCloud::ConstPtr& msg)
    {
      if (msg->width >3){
      analyser.setInputCloud(msg);
      Matrix eg = analyser.getEigenVectors();
      ROS_INFO("PointCloud received");

      float x, y, z, th, h, c;
      x = y = z = th = h = c = 0.;

      // indices : tout le PointCloud
      std::vector<int> indices;
      for (int i = 0; i < msg->points.size(); ++i)
        indices.push_back(i);



      // vérifier signature
      //estimator.computePointNormal(*msg, indices, x, y, z, c);
           //Produit vectoriel des deux première colonnes de Matrix3f
          /* x = eg(2,1)*eg(3,2)
                 - eg(3,1)*eg(2,2);
           y = eg(3,1)*eg(1,2)
                 - eg(1,1)*eg(3,2);
           z = eg(1,1)*eg(2,2)
                 - eg(2,1)*eg(1,2);*/
           Eigen::Vector3f v = eg.col(0).cross(eg.col(1));
           v.normalize();
           /*
           x = x/sqrt(x*x+y*y+z*z);
           y = x/sqrt(x*x+y*y+z*z);
           z = x/sqrt(x*x+y*y+z*z);
           */
           x = v(0); y=v(1); z=v(2);
       
      
      h = (analyser.getMean())(2);
      
      //h = altitude(msg);
      th = 
            2 * atan (eg(2,1)
                       /(1 + eg(1,1)));

      // publication
      ROS_INFO("Plan published");
      publisher.publish(to_Plan(x, y, z, h, th, c, msg->header.seq, msg->header.stamp, msg->width));
      }
    }

    Callback(ros::Publisher& pub) :
      publisher(pub) {}

  private:
    ros::Publisher publisher;
    pcl::PCA<Point> analyser;
    //pcl::NormalEstimationOMP<Point, pcl::Normal> estimator;
    
    inline
    const hand_control::Plan::ConstPtr
    to_Plan(const float& x, const float& y,
            const float& z, const float& h,
            const float& th,
            const float& c, const uint32_t& seq,
            const uint64_t& msec64, const uint64_t& number)
    {
      hand_control::Plan::Ptr ros_msg(new hand_control::Plan());
      ros_msg->normal.x = x; 
      ros_msg->normal.y = y; 
      ros_msg->normal.z = z; 
      ros_msg->altitude = h;
      ros_msg->angle = th;
      ros_msg->curvature = c; 
      ros_msg->number = number;
      // uint64_t msec64 is in ms (10-6)
      uint64_t sec64 = msec64 / 1000000;
      uint64_t nsec64 = (msec64 % 1000000) * 1000;
      ros_msg->header.stamp.sec = (uint32_t) sec64;
      ros_msg->header.stamp.nsec = (uint32_t) nsec64;
      ros_msg->header.seq = seq;
      ros_msg->header.frame_id = "0";
      return ros_msg;
    }

/*    inline
    float
    altitude(const PointCloud::ConstPtr& pcl)
    {
      int s = pcl->points.size();
      float h(0);
      for (int i = 0; i < s; ++i)
        h += pcl->points[i].z;
      return h/( (float) s );
      
      
      getmean
      getVector4fMap
    }*/
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "normal_estimator_pca");
  ros::NodeHandle node("estimator");//`A vérifier ?

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
