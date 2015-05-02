#include <ros/ros.h>
#include <time.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// for UniformGenerator
#include <pcl/common/random.h>
// for CloudGenerator
#include <pcl/common/generate.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
typedef pcl::common::UniformGenerator<float> UGenerator;

class Generator
{
  public:
    Generator(int len, double m, double M)
    : length(len), min(m), max(M), cgen(), number(0)
    {
      UGenerator::Parameters params(min, max, -1);
      cgen.setParameters(params);
    }

    PointCloud::Ptr
    operator()()
    {
      PointCloud::Ptr pcl(new PointCloud());
      cgen.fill(length, length, *pcl);
      for (int i = 0; i < pcl->points.size(); ++i)
      {
        pcl->points[i].r = (uint8_t) 255;
        pcl->points[i].g = (uint8_t) 255;
        pcl->points[i].b = (uint8_t) 0;
      }
      ros::Time now = ros::Time::now();
      pcl->header.stamp =  now.toNSec() / 1000;
      pcl->header.seq = number++;
      pcl->header.frame_id = "0";
      return pcl;
    }

  private:
    pcl::common::CloudGenerator<pcl::PointXYZRGB, UGenerator> cgen;
    int length;
    double min, max;
    uint32_t number;
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "random_pcl_publisher");
  ros::NodeHandle node("random");
  // paramètres
  double freq;
  if (node.getParam("freq", freq))
  {
    ROS_INFO("freq : %f" , freq);
  } else {
    node.setParam("freq", 10);
    node.getParam("freq", freq);
    ROS_INFO("freq : %f (default value)", freq);
  }
  double min, max;
  if (node.getParam("min", min))
  {
    ROS_INFO("min : %f" , min);
  } else {
    node.setParam("min", 0.);
    node.getParam("min", min);
    ROS_INFO("min : %f (default value)", min);
  }
  if (node.getParam("max", max))
  {
    ROS_INFO("max : %f" , max);
  } else {
    node.setParam("max", 100.);
    node.getParam("max", max);
    ROS_INFO("max : %f (default value)", max);
  }
  int length;
  if (node.getParam("length", length))
  {
    ROS_INFO("length : %d" , length);
  } else {
    node.setParam("length", 10);
    node.getParam("length", length);
    ROS_INFO("length : %d (default value)", length);
  }
  // initialisation
  ros::Publisher  publisher = node.advertise<PointCloud>("output", 1);
  Generator generator(length, min, max);
  ros::Rate loop_rate(freq);
  ROS_INFO("node started");
  while (ros::ok())
  {
    publisher.publish(generator());
    ROS_INFO("random PointCloud published");
    ros::spinOnce();
    loop_rate.sleep();
  }
  ROS_INFO("exit");
  return 0;
}
