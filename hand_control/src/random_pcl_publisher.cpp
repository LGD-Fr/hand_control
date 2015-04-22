#include <ros/ros.h>
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
    Generator(int l) : length(l), cgen()
    {
      UGenerator::Parameters params(0, 900, -1);
      cgen.setParameters(params);
    }

    PointCloud::Ptr
    operator()()
    {
      PointCloud::Ptr pcl(new PointCloud());
      cgen.fill(length, length, *pcl);
      ROS_INFO("random cloud :");
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
      return pcl;
    }

  private:
    pcl::common::CloudGenerator<pcl::PointXYZRGB, UGenerator> cgen;
    int length;
};

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "random_pcl_publisher");
  ros::NodeHandle node;
  ros::Publisher  publisher = node.advertise<PointCloud>("random_output", 1);
  Generator generator(5);
  ros::Rate loop_rate(0.5);
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
