/*!
  \file        sumo_animations.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/03/07

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the plied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

Play predefined animations for the Jumping Sumo.
Useful for rosmariokart.

\section Parameters
 */
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

ros::Publisher posture_pub, sharp_turn_pub, cmd_vel_pub;
std_msgs::String posture;
std_msgs::Float32 sharp_turn;
geometry_msgs::Twist cmd_vel;

inline void set_posture(const std::string & s) {
  posture.data = s;
  posture_pub.publish(posture);
  ros::spinOnce();
}

inline void spin(const int & speed, const double & time) {
  geometry_msgs::Twist c;
  c.angular.z = speed;
  ros::Time now = ros::Time::now();
  while ((ros::Time::now() - now).toSec() < time) {
    cmd_vel_pub.publish(c);
    ros::spinOnce();
  }
}

inline void stop_robot() {
  set_posture("jumper");
  geometry_msgs::Twist zero;
  cmd_vel_pub.publish(zero);
  ros::spinOnce();
}

void anim_cb(const std_msgs::StringConstPtr & msg) {
  std::string anim = msg->data;
  ROS_WARN("anim_cb('%s')", anim.c_str());
  if (anim == "win") {
    set_posture("standing");
    spin(70, 2);
    stop_robot();
  }
  else if (anim == "lose") {
    set_posture("kicker");
    ros::Duration(5).sleep();
    stop_robot();
  }
  else if (anim == "hit") {
    spin(70, .7);
    stop_robot();
  }
  else if (anim == "hit2") {
    set_posture("kicker");
    ros::Duration(2).sleep();
    stop_robot();
  }
} // end anim_cb();

int main(int argc, char** argv) {
  ros::init(argc, argv, "sumo_animations");
  ros::NodeHandle nh_public;
  ros::Subscriber sub = nh_public.subscribe("animation", 1, anim_cb);
  posture_pub = nh_public.advertise<std_msgs::String>("set_posture", 1);
  sharp_turn_pub = nh_public.advertise<std_msgs::Float32>("sharp_turn", 1);
  cmd_vel_pub = nh_public.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  ros::spin();
  return 0;
}
