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
#include <hand_control/CommanderConfig.h>


class Run
{
  private:
    float xx, yy, zz, theta;
    float DEMI_PI;

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
       mvt->angular.z = theta * angle_vel;
     }

      assert(mvt->linear.x == 0. || mvt->linear.y == 0.);
      pub.publish(mvt);
      ROS_INFO("cmd published");
    }//end publish

  public:
    Run(const ros::Publisher& cmd_publisher) :
      pub(cmd_publisher)
      {
        DEMI_PI = 2*atan(1.);
      }

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

	theta = msg->angle - DEMI_PI;

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

    void reconfigure(const hand_control::CommanderConfig& c, const uint32_t& level) {
      max_curv = c.max_curvature;
      x_dev_min = c.x_minimal_deviation;
      y_dev_min = c.y_minimal_deviation;
      z_dev_min = c.z_minimal_deviation;
      th_dev_min = c.theta_minimal_deviation;
      neutral_z = c.neutral_alt;
      min_number = c.min_points_number;
      up_factor = c.up_fact;
      plan_vel = c.plan_vel;
      z_vel = c.z_vel;
      angle_vel = c.angle_vel;
    }  

    void run()
    {
        ros::spin();
    }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "commander");
  ros::NodeHandle node("commander");

  ros::Publisher cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  Run run(cmd_pub);
  ros::Subscriber plan_sub = node.subscribe<hand_control::Plan>("input", 1, &Run::callback, &run);
  
  dynamic_reconfigure::Server<hand_control::CommanderConfig> server;
  dynamic_reconfigure::Server<hand_control::CommanderConfig>::CallbackType f;
  f = boost::bind(&Run::reconfigure, &run, _1, _2);
  server.setCallback(f);

  run.run();
  return 0;
}
