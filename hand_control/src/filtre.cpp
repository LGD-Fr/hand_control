#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class Callback {
  public:
    void
    operator()(const PointCloud::ConstPtr& msg)
    {
      ROS_INFO("PointCloud received");
      // pour publier un shared_ptr (mieux)
      PointCloud::Ptr pcl(new PointCloud());
      // copie du nuage de point
      *pcl = *msg;
      // filtrage
      double zmax(0.);
      ros::param::getCached("zmax", zmax);
      ROS_INFO("zmax : %f", zmax);
      z_filtre.setFilterLimits(0.0, zmax);
      z_filtre.setInputCloud(pcl);
      z_filtre.filter(*pcl);
      // publication
      publisher.publish(pcl);
      ROS_INFO("PointCloud published");
      ROS_INFO("filtered cloud :");
      for(int i = 0; i < pcl->points.size(); ++i)
      {
        ROS_INFO("\nx : %f\ny : %f\nz : %f\nr : %d\ng : %d\nb : %d",
                pcl->points[i].x,
                pcl->points[i].y,
                pcl->points[i].z,
                pcl->points[i].r,
                pcl->points[i].g,
                pcl->points[i].b);
      }
    }

    Callback(ros::Publisher& pub) : publisher(pub), z_filtre()
    {
      z_filtre.setFilterFieldName("z");
    }

  private:
    ros::Publisher publisher;
    pcl::PassThrough<pcl::PointXYZRGB> z_filtre;
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "filtre");
  ros::NodeHandle node;

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

  // initialisation
  ros::Publisher  publisher = node.advertise<PointCloud>("filtre_output", 1);
  Callback callback(publisher);
  ros::Subscriber subscriber = node.subscribe<PointCloud>("filtre_input", 1, callback);

  // démarrage
  ROS_INFO("node started");
  ros::spin();
  ROS_INFO("exit");
  return 0;
}
