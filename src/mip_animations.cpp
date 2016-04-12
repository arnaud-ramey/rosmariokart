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

inline void minisleep() { ros::Duration(.2).sleep(); }

void anim_cb(const std_msgs::StringConstPtr & msg) {
  std::string anim = msg->data;
  ROS_WARN("anim_cb('%s')", anim.c_str());
  if (anim == "hit") {
    play_sound(7 + rand() % 3); // punch 1 -> 3
    minisleep();
    set_chest_led(255, 0, 0, 0.2, 0.2); // chest blink red
    minisleep();
    ros::Rate rate(50); // 20 ms
    int speed = (rand() % 2 ? -10 : 10);
    for (int i = 0; i < 100; ++i) {
      if (i % 15 == 0)
        set_head_led((i+1)%2, i%2, i%2,(i+1)%2); // center -> outside
      set_cmd_vel(0, speed);
      rate.sleep();
    }
    minisleep();
    switch (rand()%12) {
      case 0: play_sound(4); break; // gnagnagna
      case 1: play_sound(14); break; // hey
      case 2: play_sound(15); break; // oooo
      case 3: play_sound(27); break; // oooo
      case 4: play_sound(37); break; // aie
      case 5: play_sound(54); break; // woo?
      case 6: play_sound(58); break; // woo 2
      case 7: play_sound(59); break; // woo 3
      case 8: play_sound(62); break; // woo 4
      case 9: play_sound(74); break; // oooooooooooooooo
      case 10: play_sound(78); break; // aaaa
      case 11: play_sound(97); break; // me tooo
      default: break;
    }
    minisleep();
    set_default_chest_led();
    minisleep();
    set_default_head_led();
  } // end "hit"

  else if (anim == "letsgo")
    play_sound(29); // let's go


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
    switch (rand()%16) {
      case 0: play_sound(16); break; // oh yeah
      case 1: play_sound(26); break; // yeah
      case 2: play_sound(28); break; // laugh
      case 3: play_sound(35); break; // you lose
      case 4: play_sound(39); break; // sing
      case 5: play_sound(40); break; // sing 2
      case 6: play_sound(41); break; // laugh 2
      case 7: play_sound(45); break; // make fun
      case 8: play_sound(47); break; // music 1
      case 9: play_sound(48); break; // music 2
      case 10: play_sound(52); break; // laugh 3
      case 11: play_sound(56); break; // oh yeah 2
      case 12: play_sound(65); break; // make fun 2
      case 13: play_sound(73); break; // sing 3
      case 14: play_sound(80); break; // yeah
      case 15: play_sound(91); break; // laugh 4
      default: break;
    }
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
  srand(time(NULL));
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
