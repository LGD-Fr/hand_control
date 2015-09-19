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


#include <assert.h>
#include <hand_control/FilterConfig.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/tracking/impl/hsv_color_coherence.hpp>
#include <dynamic_reconfigure/server.h>
#include <algorithm>


typedef pcl::PointXYZRGB Point;
typedef pcl::PointCloud<Point> PointCloud;

class Callback {
 public:
    // handles and filters the received PointCloud and
    // publishes the filtered PointCloud
    void callback(const PointCloud::ConstPtr& msg) {
        PointCloud::Ptr pcl(new PointCloud());
        copy_info(msg, pcl);  // copy the header
        BOOST_FOREACH(const Point& pt, msg->points) {
            float hue_dist, sat, val;
            hdist_s_v(pt, hue_dist, sat, val);
            if (pt.z < z_max &&
                hue_dist < delta_hue &&
                sat < sat_max &&
                sat > sat_min &&
                val < val_max &&
                val > val_min) {
                  pcl->push_back(pt);
            }
        }
        pcl->height = 1;
        pcl->width = pcl->points.size();
        publisher.publish(pcl);
    }

    explicit Callback(const ros::Publisher& pub) :
        publisher(pub),
        z_max(90.),
        hue(0.),
        delta_hue(20.),
        sat_min(0.3),
        sat_max(1.),
        val_min(0.3),
        val_max(1.) {}

  // updates the parameters
  void reconfigure(const hand_control::FilterConfig& c,
                   const uint32_t& level) {
        z_max = c.z_max;
        hue = c.hue;
        delta_hue = c.delta_hue;
        val_min = c.val_min;
        val_max = c.val_max;
        sat_min = c.sat_min;
        sat_max = c.sat_max;
  }

 private:
    ros::Publisher publisher;
    float z_max, hue, delta_hue, val_min,
        val_max, sat_min, sat_max;

    // copy the header info (useful in order to use rviz)
    inline void copy_info(const PointCloud::ConstPtr& a,
                          PointCloud::Ptr& b) {
        b->header = a->header;
        b->sensor_origin_ = a->sensor_origin_;
        b->sensor_orientation_ = a->sensor_orientation_;
        b->is_dense = a->is_dense;
    }

    // calculate the distance from the wished hue,
    // the saturation and the value of the point
    inline void hdist_s_v(const Point& pt,
                          float& h_dist,
                          float& s,
                          float& v) {
        float h, diff1, diff2;
        pcl::tracking::RGB2HSV(pt.r, pt.g, pt.b, h, s, v);
        h *= 360.0f;
        diff1 = std::fabs(h - hue);
        // hue is periodic
        if (h < hue)
            diff2 = std::fabs(360.0f + h - hue);
        else
            diff2 = std::fabs(360.0f + hue - h);
        h_dist = std::min(diff1, diff2);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter");
    ros::NodeHandle node("filter");

    ros::Publisher  publisher =
        node.advertise<PointCloud>("output", 1);

    Callback my_callback(publisher);

    ros::Subscriber subscriber =
        node.subscribe<PointCloud>("input", 1,
                                   &Callback::callback,
                                   &my_callback);

    // sets up dynamic_reconfigure
    dynamic_reconfigure::Server<hand_control::FilterConfig> server;
    dynamic_reconfigure::Server<hand_control::FilterConfig>::CallbackType f;
    f = boost::bind(&Callback::reconfigure, &my_callback, _1, _2);
    server.setCallback(f);

    // begins working
    ROS_INFO("node started");
    ros::spin();
    ROS_INFO("exit");
    return 0;
}
