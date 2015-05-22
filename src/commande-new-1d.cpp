#include <ros/ros.h>
#include <ros/time.h>
#include <locale.h>
#include <limits>
#include <math.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <hand_control/Plan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

class Run
{
  private:
    float xx, yy, dz;

    // xx > 0 : forward
    // xx < 0 : backward

    // yy > 0 : right
    // yy < 0 : left

    // dz > 0 : up
    // dz < 0 : down

    float plan_vel, z_vel, up_factor;

    // to calculate dz
    float z_current, z_previous;
    ros::Time t_current, t_previous;

    // conditions to publish a message
    float max_curv;
    float dz_min, x_dev_min, y_dev_min;
    uint64_t min_number;

    bool first_msg;

    ros::Publisher pub;

    void publish()
    {
      geometry_msgs::Twist::Ptr mvt(new geometry_msgs::Twist());

      if (fabs(dz) > dz_min)
      {
        if (dz > 0)
          mvt->linear.z = dz * z_vel * up_factor ;
        else
          mvt->linear.z = dz * z_vel;
      }

      if (fabs(yy) > fabs(xx) && fabs(yy) > y_dev_min)
      {
        mvt->linear.y = yy * plan_vel;
      }
      else if (fabs(xx) > x_dev_min)
      {
        mvt->linear.x = xx * plan_vel;
      }

      pub.publish(mvt);
      ROS_INFO("cmd published");
    }//end publish

  public:
    Run(const ros::Publisher& cmd_publisher, const float& max_curvature, const float& plan_velocity,
        const float& z_velocity, const float& x_minimal_deviation, const float& y_minimal_deviation,
        const float& dz_minimal_difference, const int& min_points_number, const float& up_fact) :
      pub(cmd_publisher),
      plan_vel(plan_velocity),
      max_curv(max_curvature),
      z_vel(z_velocity),
      xx(0),
      yy(0),
      dz(0),
      x_dev_min(x_minimal_deviation),
      y_dev_min(y_minimal_deviation),
      dz_min(dz_minimal_difference),
      first_msg(true),
      min_number(min_points_number),
      up_factor(up_fact) {
        z_current = z_previous =  std::numeric_limits<float>::signaling_NaN();
        t_previous.nsec = t_previous.sec =
          t_previous.nsec = t_previous.sec = std::numeric_limits<uint32_t>::signaling_NaN();
      }

    void callback(const hand_control::Plan::ConstPtr& msg)
    {
      ROS_INFO("plan received");
      if (msg->curvature < max_curv && msg->number > min_number)
      {
        t_current = msg->header.stamp;
        z_current = msg->altitude;

        if (!first_msg)
        {
          dz = (z_current - z_previous)/((t_current - t_previous).toSec());
          ROS_INFO("dz = %f", dz);
        }
        
        if(msg->normal.z > 0)
        {
          yy = msg->normal.x;
          xx = msg->normal.y;
        }
        else
        {
          yy = - msg->normal.x;
          xx = - msg->normal.y;
        }

        t_previous = t_current;
        z_previous = z_current;
        z_current = std::numeric_limits<float>::signaling_NaN();
        t_current.nsec = t_current.sec = std::numeric_limits<uint32_t>::signaling_NaN();
        if (first_msg)
        {
          first_msg = false;
          ROS_INFO("first msg received");
        }
        ROS_INFO("coords updated");
      } else {
        xx = yy = dz = 0.;
      }
      publish();
    };

    void run()
    {
        ros::spin();
    }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "commande");
  ros::NodeHandle node("commande");

  double max_curv(0);
  if (node.getParam("max_curv", max_curv))
  {
    ROS_INFO("max_curv : %f" , max_curv);
  } else {
    node.setParam("max_curv", 0.08);
    node.getParam("max_curv", max_curv);
    ROS_INFO("max_curv : %f (default value)", max_curv);
  }

  double plan_vel(0);
  if (node.getParam("plan_vel", plan_vel))
  {
    ROS_INFO("plan_vel : %f" , plan_vel);
  } else {
    node.setParam("plan_vel", 0.8);
    node.getParam("plan_vel", plan_vel);
    ROS_INFO("plan_vel : %f (default value)", plan_vel);
  }

  double z_vel(0);
  if (node.getParam("z_vel", z_vel))
  {
    ROS_INFO("z_vel : %f" , z_vel);
  } else {
    node.setParam("z_vel", 0.8);
    node.getParam("z_vel", z_vel);
    ROS_INFO("z_vel : %f (default value)", z_vel);
  }

  int min_number(0);
  if (node.getParam("min_number", min_number))
  {
    ROS_INFO("min_number : %d" , min_number);
  } else {
    node.setParam("min_number", 500);
    node.getParam("min_number", min_number);
    ROS_INFO("min_number : %d (default value)", min_number);
  }

  double x_dev_min(0);
  if (node.getParam("x_dev_min", x_dev_min))
  {
    ROS_INFO("x_dev_min : %f" , x_dev_min);
  } else {
    node.setParam("x_dev_min", 0.05);
    node.getParam("x_dev_min", x_dev_min);
    ROS_INFO("x_dev_min : %f (default value)", x_dev_min);
  }

  double y_dev_min(0);
  if (node.getParam("y_dev_min", y_dev_min))
  {
    ROS_INFO("y_dev_min : %f" , y_dev_min);
  } else {
    node.setParam("y_dev_min", 0.05);
    node.getParam("y_dev_min", y_dev_min);
    ROS_INFO("y_dev_min : %f (default value)", y_dev_min);
  }

  double dz_dev_min(0);
  if (node.getParam("dz_dev_min", dz_dev_min))
  {
    ROS_INFO("dz_dev_min : %f" , dz_dev_min);
  } else {
    node.setParam("dz_dev_min", 0.05);
    node.getParam("dz_dev_min", dz_dev_min);
    ROS_INFO("dz_dev_min : %f (default value)", dz_dev_min);
  }

  double up_fact(0);
  if (node.getParam("up_fact", up_fact))
  {
    ROS_INFO("up_fact : %f" , up_fact);
  } else {
    node.setParam("up_fact", 1.5);
    node.getParam("up_fact", up_fact);
    ROS_INFO("up_fact : %f (default value)", up_fact);
  }

  ros::Publisher cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  Run run(cmd_pub, max_curv, plan_vel, z_vel, x_dev_min, y_dev_min, dz_dev_min, min_number, up_fact);
  ros::Subscriber plan_sub = node.subscribe<hand_control::Plan>("input", 1, &Run::callback, &run);
  run.run();
  return 0;
}