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
#include <hand_control/Commander_atanConfig.h>
#include "data.h"

class Run
{
  private:
    class Function
    {
      private:
        float t;
        static const float demi_pi;
      public:
        float operator()(float arg)
        {
          return atan(t * arg) / demi_pi;
        }
        Function() : t(demi_pi){}
        void set_t(float tt)
        {
          t = tt;
        }
        void set_p(float pp)
          // pp : gradient in 0
        {
          t = pp*demi_pi;
        }
    };

    Data<Function> f;
    Data<float> min;
    float neutral_z;
    float max_curv;
    uint64_t min_number;
    ros::Publisher pub;

  public:
    Run(const ros::Publisher& cmd_publisher) :
      pub(cmd_publisher) {
        f.x = f.y = f.z = f.th = Function();
        min.x = min.y = min.z = min.th = 0.1;
      }

    void callback(const hand_control::Plan::ConstPtr& msg)
    {
      ROS_INFO("plan received");
      Data<float> in, out;
      in.x = in.y = in.z = in.th = 0.;
      out.x = out.y = out.z = out.th = 0;
      if (msg->curvature < max_curv && msg->number > min_number)
      {
        if(msg->normal.z > 0)
        {
          in.y = msg->normal.x;
          in.x = msg->normal.y;
        } else {
          in.y = - msg->normal.x;
          in.x = - msg->normal.y;
        }
        in.z = msg->altitude - neutral_z;
        in.th = msg->angle;
      }

      geometry_msgs::Twist::Ptr mvt(new geometry_msgs::Twist());

      if (fabs(in.y) > fabs(in.x))
      {
        out.y = f.y(in.y);
        if (out.y > min.y)
          mvt->linear.y = out.y;
      } else {      {
        out.x = f.x(in.x);
        if (out.x > min.x)
          mvt->linear.x = out.x;
      }

      out.z = f.z(in.z);
      if (out.z > min.z)
        mvt->linear.z = out.z;

      out.th = f.th(in.th);
      if (out.th > min.th)
        mvt->angular.z = out.th;

      pub.publish(mvt);
      ROS_INFO("cmd published");
      }
    }

    void reconfigure(const hand_control::Commander_atanConfig& c, const uint32_t& level)
    {
      max_curv = c.max_curvature;
      neutral_z = c.neutral_alt;
      min_number = c.min_points_number;
      f.x.set_p(c.p_x);
      f.y.set_p(c.p_y);
      f.z.set_p(c.p_z);
      f.th.set_p(c.p_th);
      min.x = c.x_min;
      min.y = c.y_min;
      min.z = c.z_min;
      min.th = c.th_min;
    }  

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "commander_atan");
  ros::NodeHandle node("commander_atan");

  ros::Publisher cmd_pub = node.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  Run run(cmd_pub);
  ros::Subscriber plan_sub = node.subscribe<hand_control::Plan>("input", 1, &Run::callback, &run);

  dynamic_reconfigure::Server<hand_control::Commander_atanConfig> server;
  dynamic_reconfigure::Server<hand_control::Commander_atanConfig>::CallbackType f;
  f = boost::bind(&Run::reconfigure, &run, _1, _2);
  server.setCallback(f);

  ROS_INFO("start spinning");
  ros::spin();
  return 0;
}

const float Run::Function::demi_pi = 2*atan(1.);
