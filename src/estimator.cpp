/* Copyright © 2015 CentraleSupélec
 *

 * This file is part of Hand Control.
 *

 * Hand Control is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *

 * Hand Control is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *

 * You should have received a copy of the GNU General Public License
 * along with Hand Control.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d_omp.h>
#include <hand_control/Plan.h>
#include <time.h>
#include <math.h>
#include <dynamic_reconfigure/server.h>
#include <hand_control/EstimatorConfig.h>

#include <pcl/common/pca.h>

typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef Eigen::Matrix3f& Matrix;

class Callback {
 public:
    // handles received messages and publish the plan estimation
    void callback(const PointCloud::ConstPtr& msg) {
        ROS_INFO("PointCloud received");

        if (msg->width > 3) {
            // else, no plan can be estimated…
            analyser.setInputCloud(msg);
            Matrix eg = analyser.getEigenVectors();

            // to build the "Plan" message to be published
            float x, y, z, th, h, c;
            x = y = z = th = h = c = 0.;

            // v = eg_1 ^ eg_2 is the plan normal
            Eigen::Vector3f v = eg.col(0).cross(eg.col(1));
            v.normalize();  // to have norm(v) == 1

            // x, y and z are the coords of the plan normal
            if (!reverse) {
                x = v(0); y = v(1);
            } else {
                // if "x" and "y" axes are inverted
                // (reverse is a parameter to set with dynamic_reconfigure)
                x = v(1); y = v(0);
            }
            z = v(2);

            // h is the altitude
            h = (analyser.getMean())(2);


            // angle calculation


            // m_x and m_y are the "x" and "y" coords
            // of the first principal component
            float m_x, m_y;



            // parameter to set
            // with dynamic_reconfigure
            if (reverse_angle) {
                m_x = eg(0, 0);
                m_y = eg(1, 0);
            } else {
                m_x = eg(1, 0);
                m_y = eg(0, 0);
            }

            // because we want "th" only between -90° and 90°
            if (m_x < 0.)
                m_y *= -1;

            th = - asin(m_y / sqrt(pow(m_y, 2)+ pow(m_x, 2)));  // 0 <= th <= pi
            th *= _RAD2DEG;  // -90 <= th <= 90

            // TODO(someone)
            // -> calculate "c" (the curvature)
            // ( c == 0 for the moment)

            // publication
            ROS_INFO("Plan published");
            publisher.publish(
                to_Plan(
                    x, y, z, h, th, c,
                    msg->header.seq,
                    msg->header.stamp,
                    msg->width));
        }
    }

    explicit Callback(const ros::Publisher& pub) :
        publisher(pub),
        _RAD2DEG(45.f/atan(1.)),
        reverse(false),
        reverse_angle(false) {}

    // updates the parameters received from dynamic_reconfigure
    void reconfigure(const hand_control::EstimatorConfig& c,
                     const uint32_t& level) {
        reverse = c.reverse;
        reverse_angle = c.reverse_angle;
    }

 private:
    ros::Publisher publisher;
    pcl::PCA<Point> analyser;
    const float _RAD2DEG;
    bool reverse, reverse_angle;

    // return a "Plan" message build with
    // the informations provided
    inline const hand_control::Plan::ConstPtr
        to_Plan(const float& x, const float& y,
                const float& z, const float& h,
                const float& th,
                const float& c, const uint32_t& seq,
                const uint64_t& msec64, const uint64_t& number) {
            hand_control::Plan::Ptr ros_msg(new hand_control::Plan());
            ros_msg->normal.x = x;
            ros_msg->normal.y = y;
            ros_msg->normal.z = z;

            ros_msg->altitude = h;
            ros_msg->angle = th;
            ros_msg->curvature = c;

            ros_msg->number = number;
            uint64_t sec64 = msec64 / 1000000;
            uint64_t nsec64 = (msec64 % 1000000) * 1000;
            ros_msg->header.stamp.sec = (uint32_t) sec64;
            ros_msg->header.stamp.nsec = (uint32_t) nsec64;
            ros_msg->header.seq = seq;
            ros_msg->header.frame_id = "0";
            return ros_msg;
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "estimator");
    ros::NodeHandle node("estimator");
    ros::Publisher  publisher = node.advertise<hand_control::Plan>("output", 1);
    Callback callback(publisher);
    ros::Subscriber subscriber =
        node.subscribe<PointCloud>("input", 1, &Callback::callback, &callback);

    // sets up dynamic_reconfigure
    dynamic_reconfigure::Server<hand_control::EstimatorConfig> server;
    dynamic_reconfigure::Server<hand_control::EstimatorConfig>::CallbackType f;
    f = boost::bind(&Callback::reconfigure, &callback, _1, _2);
    server.setCallback(f);

    // begins working
    ROS_INFO("node started");
    ros::spin();
    ROS_INFO("exit");
    return 0;
}
