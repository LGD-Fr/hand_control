#include <ros/ros.h>
#include <ros/time.h>
#include <locale.h>
#include <limits>
#include <math.h>
#include <assert.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <hand_control/Plan.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <dynamic_reconfigure/server.h>
#include <hand_control/CommandeConfig.h>


class Run
{
  private:
    float xx, yy, zz, theta;

    // xx < 0 : forward
    // xx > 0 : backward

    // yy > 0 : right
    // yy < 0 : left

    // zz > 0 : up
    // zz < 0 : down

    float plan_vel, z_vel, angle_vel, up_factor, neutral_z;

    float max_curv;
    float z_dev_min, x_dev_min, y_dev_min, th_dev_min;
    uint64_t min_number;

    bool first_msg;

    ros::Publisher pub;

    void publish()
    {
      geometry_msgs::Twist::Ptr mvt(new geometry_msgs::Twist());

      if (fabs(zz) > z_dev_min)
      {
        if (zz > 0)
          mvt->linear.z = zz * z_vel * up_factor ;
        else
          mvt->linear.z = zz * z_vel;
      }

      if (fabs(yy) > fabs(xx) && fabs(yy) > y_dev_min)
      {
        mvt->linear.y = yy * plan_vel;
      }
      else if (fabs(xx) > x_dev_min)
      {
        mvt->linear.x = - xx * plan_vel;
      }
      
     if (fabs(theta) > th_dev_min) {
       mvt->angular.z * angle_vel;
     }

      assert(mvt->linear.x == 0. || mvt->linear.y == 0.);
      pub.publish(mvt);
      ROS_INFO("cmd published");
    }//end publish

  public:
    Run(const ros::Publisher& cmd_publisher, const float& max_curvature, const float& plan_velocity, const float& angle_velocity,
        const float& z_velocity, const float& x_minimal_deviation, const float& y_minimal_deviation,
        const float& z_minimal_deviation, const float& neutral_alt, const float& theta_minimal_deviation, 
	const int& min_points_number, const float& up_fact) :
      pub(cmd_publisher),
      plan_vel(plan_velocity),
      angle_vel(angle_velocity);
      max_curv(max_curvature),
      z_vel(z_velocity),
      xx(0),
      yy(0),
      zz(0),
      theta(0),
      x_dev_min(x_minimal_deviation),
      y_dev_min(y_minimal_deviation),
      z_dev_min(z_minimal_deviation),
      neutral_z(neutral_alt),
      first_msg(true),
      min_number(min_points_number),
      up_factor(up_fact)

    void callback(const hand_control::Plan::ConstPtr& msg)
    {
      ROS_INFO("plan received");
      if (msg->curvature < max_curv && msg->number > min_number)
      {
        
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

        zz = msg->altitude - neutral_z;

	theta = msg->angle;

        if (first_msg)
        {
          first_msg = false;
          ROS_INFO("first msg received");
        }
        ROS_INFO("coords updated");
      } else {
        xx = yy = zz = 0.;
      }
      publish();
    };

    void reconfigure(const hand_control::CommandeConfig& c, const uint32_t& level) {
      max_curv = c.max_curvature;
      x_dev_min = c.x_minimal_deviation;
      y_dev_min = c.y_minimal_deviation;
      z_dev_min = c.z_minimal_deviation;
      th_dev_min = c.theta_minimal_deviation;
      neutral_z = c.neutral_alt;
      min_number = c.min_points_number;
      up_factor = c.up_fact;
    }  

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

  double angle_vel(0);
  if (node.getParam("angle_vel", angle_vel))
  {
    ROS_INFO("angle_vel : %f" , angle_vel);
  } else {
    node.setParam("angle_vel", 10);
    node.getParam("angle_vel", angle_vel);
    ROS_INFO("angle_vel : %f (default value)", angle_vel);
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

  double z_dev_min(0);
  if (node.getParam("z_dev_min", z_dev_min))
  {
    ROS_INFO("z_dev_min : %f" , z_dev_min);
  } else {
    node.setParam("z_dev_min", 0.1);
    node.getParam("z_dev_min", z_dev_min);
    ROS_INFO("z_dev_min : %f (default value)", z_dev_min);
  }

  double th_dev_min(0);
  if (node.getParam("th_dev_min", th_dev_min))
  {
    ROS_INFO("th_dev_min : %f" , th_dev_min);
  } else {
    node.setParam("th_dev_min", 15);
    node.getParam("th_dev_min", th_dev_min);
    ROS_INFO("th_dev_min : %f (default value)", th_dev_min);
  }

  double neutral_z(0);
  if (node.getParam("neutral_z", neutral_z))
  {
    ROS_INFO("neutral_z : %f" , neutral_z);
  } else {
    node.setParam("neutral_z", 1.5);
    node.getParam("neutral_z", neutral_z);
    ROS_INFO("neutral_z : %f (default value)", neutral_z);
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
  Run run(cmd_pub, max_curv, plan_vel, z_vel, angle_vel, x_dev_min, y_dev_min, z_dev_min, th_dev_min, neutral_z, min_number, up_fact);
  ros::Subscriber plan_sub = node.subscribe<hand_control::Plan>("input", 1, &Run::callback, &run);
  
  dynamic_reconfigure::Server<hand_control::CommandeConfig> server;
  dynamic_reconfigure::Server<hand_control::CommandeConfig>::CallbackType f;
  f = boost::bind(&Run::reconfigure, &run, _1, _2);
  server.setCallback(f);

  run.run();
  return 0;
}
