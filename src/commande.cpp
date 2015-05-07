#include <ros/ros.h>
#include <ros/time.h>
#include <locale.h>
#include <limits>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <hand_control/Plan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

class Run
{
  private:
    ros::Rate loop_rate;

    // xx*plan_vel, yy*plan_vel and dz*dz_vel will be published
    double xx, yy, dz;
    double plan_vel, z_vel;

    // to calculate dz
    double z_current, z_previous;
    ros::Time t_current, t_previous;

    // conditions to publish a message
    double max_curv;
    double dz_min, x_dev_min, y_dev_min;

    bool first_msg;

    ros::Publisher pub;

  public:
    Run(ros::Publisher cmd_pub, double M_curve, double p_vel, double h_vel, double x_dev_m, double y_dev_m, double dz_m) :
      pub(cmd_pub),
      plan_vel(p_vel),
      max_curv(M_curve),
      z_vel(h_vel),
      loop_rate(30),
      xx(0),
      yy(0),
      dz(0),
      x_dev_min(x_dev_m),
      y_dev_min(y_dev_m),
      dz_min(dz_m),
      first_msg(true) {
      z_current = z_previous =  std::numeric_limits<double>::signaling_NaN();
      t_previous.nsec = t_previous.sec =
      t_previous.nsec = t_previous.sec = std::numeric_limits<uint32_t>::signaling_NaN();
    }


    void operator()(const hand_control::Plan::ConstPtr& msg)
    {
      ROS_INFO("plan received");
      if (msg->curvature < max_curv)
      {
        t_current = msg->header.stamp;
        z_current = msg->normal.z;
        
        if (!first_msg)
        {
          dz = (z_current - z_previous)/((t_current - t_previous).toSec());
          ROS_INFO("dz = %f", dz);
        }


        xx = msg->normal.x;
        ROS_INFO("xx = %f", xx);
        yy = msg->normal.y;
        ROS_INFO("yy = %f", yy);

        t_previous = t_current;
        z_previous = z_current;
        z_current = std::numeric_limits<double>::signaling_NaN();
        t_current.nsec = t_current.sec = std::numeric_limits<uint32_t>::signaling_NaN();
        if (first_msg)
        {
          first_msg = false;
          ROS_INFO("first msg received");
        }
        ROS_INFO("coords updated");
      }
    };

    void run()
    {
      ROS_INFO("node running");

      while(ros::ok())
      {
        ROS_INFO("loop entered");

        geometry_msgs::Twist::Ptr mvt(new geometry_msgs::Twist());
        
        ROS_INFO("dz : %f > %f : dz_min ?", dz, dz_min);
        if (dz > dz_min)
        {
          mvt->linear.z = dz * z_vel;
          ROS_INFO("z changed");
        }
        ROS_INFO("xx : %f > %f : x_dev_min ?", xx, x_dev_min);
        if (xx > x_dev_min)
        {
          mvt->linear.x = xx * plan_vel;
          ROS_INFO("x changed");
        }
        ROS_INFO("yy : %f > %f : y_dev_min ?", yy, y_dev_min);
        if (yy > y_dev_min)
        {
          mvt->linear.y = yy * plan_vel;
          ROS_INFO("y changed");
        }

        pub.publish(mvt);
        ros::spinOnce();
        loop_rate.sleep();
      }//end while
    }//end run

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
    node.setParam("max_curv", 0.1);
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

  ros::Publisher cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  Run run(cmd_pub, max_curv, plan_vel, z_vel, 0.0, 0.0, 0.0);
  ros::Subscriber plan_sub = node.subscribe<hand_control::Plan>("input", 1, run);
  run.run();
  return 0;
}
