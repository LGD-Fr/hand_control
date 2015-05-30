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

    enum FType { f_lin, f_atan, f_undef };

    class Function
    {
      private:
        virtual void set_grad(const float& grad) = 0;
      protected:
        float min, max;
        FType type;
      public:
        Function() : min(0.1f), max(0.8f) {}
        virtual float f(const float& arg) = 0;
        void set(const float& minimum, const float& maximum, const float& grad)
        {
          min = minimum;
          max = maximum;
          set_grad(grad);
        }
        FType get_type() { return type; }
    };

    class Atan : public Function
    {
      private:
        static const float demi_pi;
        float t;
        virtual void set_grad(const float& grad)
        {
          t = grad*demi_pi/max;
        }
      public:
        Atan() :t(demi_pi) { type = f_atan; }
        virtual float f(const float& arg)
        {
          float out = max * atan(t * arg) / demi_pi;
          if (fabs(out) > min)
            return out;
          else
            return 0.;
        }
    };

    class Lin : public Function
    {
      private:
        float t;
        virtual void set_grad(const float& grad)
        {
          t = grad;
        }
      public:
        Lin() { type = f_lin; }
        virtual float f(const float& arg)
        {
          float out = std::min(t * arg, max);
          if (fabs(out) > min)
            return out;
          else
            return 0.;
        }
    };

    Data<boost::shared_ptr<Function> > f;
    float neutral_z;
    float max_curv;
    uint64_t min_number;
    ros::Publisher pub;

  public:
    Run(const ros::Publisher& cmd_publisher) :
      pub(cmd_publisher) {
        f.x = boost::make_shared<Lin>();
        f.y = boost::make_shared<Lin>();
        f.z = boost::make_shared<Lin>();
        f.th = boost::make_shared<Lin>();
      }

    void callback(const hand_control::Plan::ConstPtr& msg)
    {
      ROS_INFO("plan received");
      Data<float> in, out;
      in.x = in.y = in.z = in.th = 0.;
      if (msg->curvature < max_curv && msg->number > min_number)
      {
        if(msg->normal.z > 0)
        {
          in.y = msg->normal.x;
          in.x = msg->normal.y;
        } else
        {
          in.y = - msg->normal.x;
          in.x = - msg->normal.y;
        }
        in.z = msg->altitude - neutral_z;
        in.th = msg->angle;
      }

      geometry_msgs::Twist::Ptr mvt(new geometry_msgs::Twist());

      if (fabs(in.y) > fabs(in.x))
      {
        mvt->linear.y =  f.y->f(in.y);
      } else
      {
        mvt->linear.x = f.x->f(in.x);
      }

      mvt->linear.z = f.z->f(in.z);

      mvt->angular.z = f.th->f(in.th);

      pub.publish(mvt);
      ROS_INFO("cmd published");
    }

    void reconfigure(const hand_control::Commander_atanConfig& c, const uint32_t& level)
    {
      max_curv = c.max_curvature;
      neutral_z = c.neutral_alt;
      min_number = c.min_points_number;

      if (c.atan and f.x->get_type() != f_atan)
      {
        f.x = boost::make_shared<Atan>();
        f.y = boost::make_shared<Atan>();
        f.z = boost::make_shared<Atan>();
        f.th = boost::make_shared<Atan>();
      } else if (!c.atan and f.x->get_type() == f_atan)
      {
        f.x = boost::make_shared<Lin>();
        f.y = boost::make_shared<Lin>();
        f.z = boost::make_shared<Lin>();
        f.th = boost::make_shared<Lin>();
      }
      f.x->set(c.x_min, c.x_max, c.x_p);
      f.y->set(c.y_min, c.y_max, c.y_p);
      f.z->set(c.z_min, c.z_max, c.z_p);
      f.th->set(c.th_min, c.th_max, c.th_p);
      ROS_INFO("parameters reconfigured");
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

const float Run::Atan::demi_pi = 2*atan(1);
