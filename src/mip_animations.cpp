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
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <boost/assign/std/vector.hpp>
using namespace boost::assign; // https://stackoverflow.com/questions/2236197/what-is-the-easiest-way-to-initialize-a-stdvector-with-hardcoded-elements

ros::Publisher sound_pub, cmd_vel_pub, sharp_turn_pub,
chest_led_pub, chest_led_blink_pub, head_led_pub;
std_msgs::Int16 sound;
std_msgs::Float32MultiArray sharp_turn;
std_msgs::UInt8MultiArray chest_led, head_led;
std_msgs::Float32MultiArray chest_led_blink;
geometry_msgs::Twist cmd_vel;

inline void set_chest_led(int r, int g, int b, double ton, double toff) {
  chest_led_blink.data.clear();
  chest_led_blink.data += r, g, b, ton, toff;
  chest_led_blink_pub.publish(chest_led_blink);
  ros::spinOnce();
}
inline void set_chest_led(int r, int g, int b) {
  ROS_WARN("set_chest_led(%i, %i, %i)", r, g, b);
  chest_led.data.clear();
  chest_led.data += r, g, b;
  chest_led_pub.publish(chest_led);
  ros::spinOnce();
}
inline void set_default_chest_led() { set_chest_led(0, 255, 0); }

inline void set_head_led(int l1, int l2, int l3, int l4) {
  head_led.data.clear();
  head_led.data += l1, l2, l3, l4;
  head_led_pub.publish(head_led);
  ros::spinOnce();
}
inline void set_default_head_led() { set_head_led(1, 1, 1, 1); }

inline void play_sound(int idx) {
  sound.data = idx;
  sound_pub.publish(sound);
  ros::spinOnce();
}

inline void set_cmd_vel(const double v, const double w) {
  cmd_vel.linear.x = v;
  cmd_vel.angular.z = w;
  cmd_vel_pub.publish(cmd_vel);
  ros::spinOnce();
}

inline void set_sharp_turn(const double angle, const double speed) {
  sharp_turn.data.clear();
  sharp_turn.data += angle, speed;
  sharp_turn_pub.publish(sharp_turn);
  ros::spinOnce();
}

inline void minisleep() { ros::Duration(.2).sleep(); }

void anim_cb(const std_msgs::StringConstPtr & msg) {
  std::string anim = msg->data;
  ROS_WARN("anim_cb('%s')", anim.c_str());
  if (anim == "hit") {
    set_sharp_turn(M_PI * 3, 15);
    minisleep();
    play_sound(7 + rand() % 3); // punch 1 -> 3
    minisleep();
    set_chest_led(255, 0, 0, 0.2, 0.2); // chest blink red
    minisleep();
    for (unsigned int i = 0; i < 7; ++i) {
      set_head_led((i+1)%2, i%2, i%2,(i+1)%2); // center -> outside
      usleep(300 * 1000);
    }
    set_default_chest_led();
    minisleep();
    set_default_head_led();
  } // end "hit"

  else if (anim == "lose") {
    play_sound(15); // ooooh!
    minisleep();
    set_chest_led(255, 0, 0, 0.2, 0.2); // chest blink red
    for (unsigned int i = 0; i < 7; ++i) {
      set_head_led((i+1)%2, i%2, i%2,(i+1)%2); // center -> outside
      usleep(300 * 1000);
    }
    set_default_chest_led();
    minisleep();
    set_default_head_led();
  } // end "lose"

  else if (anim == "mock"
           || anim == "win") {
    play_sound(16); // oh yeah
    minisleep();
    set_chest_led(0, 255, 0, 0.2, 0.2); // chest blink green
    for (unsigned int i = 0; i < 7; ++i) {
      set_head_led(i%2, (i+1)%2, i%2,(i+1)%2); // left -> right
      usleep(300 * 1000);
    }
    set_default_chest_led();
    minisleep();
    set_default_head_led();
  } // end "win"
} // end anim_cb();

int main(int argc, char** argv) {
  ros::init(argc, argv, "mip_animations");
  ros::NodeHandle nh_public;
  ros::Subscriber sub = nh_public.subscribe("animation", 1, anim_cb);
  sound_pub = nh_public.advertise<std_msgs::Int16>("sound", 1);
  cmd_vel_pub = nh_public.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  sharp_turn_pub = nh_public.advertise<std_msgs::Float32MultiArray>("sharp_turn_speed", 1);
  chest_led_pub = nh_public.advertise<std_msgs::UInt8MultiArray>("chest_led", 1);
  chest_led_blink_pub = nh_public.advertise<std_msgs::Float32MultiArray>("chest_led_blink", 1);
  head_led_pub = nh_public.advertise<std_msgs::UInt8MultiArray>("head_led", 1);
  ros::spin();
  return 0;
}
