/*!
  \file        generic_joy_cmd_vel  .cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/1/18

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

A simple node for teleoperating the Sumo
 */
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

int axis_linear = -1, axis_angular = -1, min_naxes_exp;
double scale_linear = 1.0, scale_angular = 1.0;
double offset_linear = 0.0, offset_angular = 0.0;
ros::Publisher cmd_vel_pub;

////////////////////////////////////////////////////////////////////////////////

void joy_cb(const sensor_msgs::Joy::ConstPtr& joy) {
  int naxes = joy->axes.size();
  if (naxes <= min_naxes_exp) {
    ROS_WARN_THROTTLE(1, "Got a joy message with %i axes while expecting at least %i",
                      naxes, min_naxes_exp);
    return;
  }
  geometry_msgs::Twist vel;
  if (0 <= axis_linear)
    vel.linear.x = ((joy->axes[axis_linear]-offset_linear) * scale_linear);
  if (0 <= axis_angular)
    vel.angular.z = ((joy->axes[axis_angular]-offset_angular) * scale_angular);
  cmd_vel_pub.publish(vel);
} // end joy_cb();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "generic_joy_cmd_vel");
  ros::NodeHandle nh_public;
  // params
  nh_public.param("axis_angular", axis_angular, axis_angular);
  nh_public.param("axis_linear", axis_linear, axis_linear);
  nh_public.param("offset_angular", offset_angular, offset_angular);
  nh_public.param("offset_linear", offset_linear, offset_linear);
  nh_public.param("scale_angular", scale_angular, scale_angular);
  nh_public.param("scale_linear", scale_linear, scale_linear);
  min_naxes_exp = std::max(axis_linear, axis_angular) + 1;
  // subscribers
  ros::Subscriber joy_sub = nh_public.subscribe<sensor_msgs::Joy>("joy", 1,  joy_cb);
  // publishers
  cmd_vel_pub = nh_public.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::spin();
  return 0;
}


