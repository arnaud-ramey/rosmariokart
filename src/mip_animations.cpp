/*!
  \file        mip_animations.cpp
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

Play predefined animations for the WowWee MIP.
Useful for rosmariokart.

\section Parameters
 */
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int8MultiArray.h>
#include <ros/ros.h>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign; // https://stackoverflow.com/questions/2236197/what-is-the-easiest-way-to-initialize-a-stdvector-with-hardcoded-elements

ros::Publisher sound_pub, sharp_turn_pub, cmd_vel_pub,
chest_led_pub, chest_led_blink_pub;

void anim_cb(const std_msgs::StringConstPtr & msg) {
  std::string anim = msg->data;
  ROS_WARN("anim_cb('%s')", anim.c_str());
  std_msgs::Int16 sound;
  std_msgs::Float32 sharp_turn;
  std_msgs::Int8MultiArray chest_led, default_chest_led;
  default_chest_led.data += 0, 255, 0;
  std_msgs::Float32MultiArray chest_led_blink;
  geometry_msgs::Twist cmd_vel;
  if (anim == "win") {
    chest_led_blink.data += 0, 255, 0, 0.2, 0.2;
    chest_led_blink_pub.publish(chest_led_blink);
    ros::spinOnce();
    //usleep(50 * 1000);
    sound.data = 16;
    sound_pub.publish(sound);
    ros::spinOnce();
    //sleep(1000 * 1000);
    chest_led_pub.publish(default_chest_led );
  }
} // end anim_cb();

int main(int argc, char** argv) {
  ros::init(argc, argv, "mip_animations");
  ros::NodeHandle nh_public;
  ros::Subscriber sub = nh_public.subscribe("animation", 1, anim_cb);
  sound_pub = nh_public.advertise<std_msgs::Int16>("sound", 1);
  sharp_turn_pub = nh_public.advertise<std_msgs::Float32>("sharp_turn", 1);
  cmd_vel_pub = nh_public.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  chest_led_pub = nh_public.advertise<std_msgs::Int8MultiArray>("chest_led", 1);
  chest_led_blink_pub = nh_public.advertise<std_msgs::Float32MultiArray>("chest_led_blink", 1);
  ros::spin();
  return 0;
}
