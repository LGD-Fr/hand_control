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
      protected:
        float max;
      public:
        virtual float f(const float& arg) = 0;
        virtual void set(const float& maximum, const float& grad) = 0;
    };

    class Atan : public Function
    {
      private:
        static const float demi_pi;
        float t;
      public:
        virtual float f(const float& arg)
        {
          return max * atan(t * arg) / demi_pi;
        }
        Atan(const float& max, const float& grad)
          :t(demi_pi)
        {
          set(max, grad);
        }
        virtual void set(const float& maximum, const float& grad)
        {
          max = maximum;
          t = grad*demi_pi/max;
        }
    };

    class Lin : public Function
    {
      private:
        float t;
      public:
        Lin(const float& max, const float& grad)
        {
          set(max, grad);
        }
        virtual void set(const float& maximum, const float& grad)
        {
          max = maximum;
          t = grad;
        }
        virtual float f(const float& arg)
        {
          return std::max(t * arg, max);
        }
    };

    enum FType { f_lin, f_atan };
    Data<boost::shared_ptr<Function> > f;
    Data<FType> f_type;
    Data<float> min;
    float neutral_z;
    float max_curv;
    uint64_t min_number;
    ros::Publisher pub;

  public:
    Run(const ros::Publisher& cmd_publisher) :
      pub(cmd_publisher) {
        f.x = boost::make_shared<Lin>(0.8f, 0.5f);
        f.y = boost::make_shared<Lin>(0.8f, 0.5f);
        f.z = boost::make_shared<Lin>(0.8f, 2.0f);
        f.th = boost::make_shared<Lin>(0.8f, 0.005f);
        f_type.x = f_type.y = f_type.z = f_type.th = f_lin;
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
        out.y = f.y->f(in.y);
        if (fabs(out.y) > min.y)
          mvt->linear.y = out.y;
      } else {
        out.x = f.x->f(in.x);
        if (fabs(out.x) > min.x)
          mvt->linear.x = out.x;
      }

      out.z = f.z->f(in.z);
      if (fabs(out.z) > min.z)
        mvt->linear.z = out.z;

      out.th = f.th->f(in.th);
      if (fabs(out.th) > min.th)
        mvt->angular.z = out.th;

      pub.publish(mvt);
      ROS_INFO("cmd published");
    }

    void reconfigure(const hand_control::Commander_atanConfig& c, const uint32_t& level)
    {
      max_curv = c.max_curvature;
      neutral_z = c.neutral_alt;
      min_number = c.min_points_number;

      if (c.atan and f_type.x != f_atan)
      {
        f.x = boost::make_shared<Atan>(0.8f, 0.5f);
        f.y = boost::make_shared<Atan>(0.8f, 0.5f);
        f.z = boost::make_shared<Atan>(0.8f, 2.f);
        f.th = boost::make_shared<Atan>(0.8f, 0.005f);
        f_type.x = f_type.y = f_type.z = f_type.th = f_atan;
        
      } else if (!c.atan and f_type.x == f_atan) {

        f.x = boost::make_shared<Lin>(0.8f, 0.5f);
        f.y = boost::make_shared<Lin>(0.8f, 0.5f);
        f.z = boost::make_shared<Lin>(0.8f, 2.0f);
        f.th = boost::make_shared<Lin>(0.8f, 0.005f);
        f_type.x = f_type.y = f_type.z = f_type.th = f_lin;
      }
      f.x->set(c.max_x, c.p_x);
      f.y->set(c.max_y, c.p_y);
      f.z->set(c.max_z, c.p_z);
      f.th->set(c.max_th, c.p_th);
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

const float Run::Atan::demi_pi = 2*atan(1);
