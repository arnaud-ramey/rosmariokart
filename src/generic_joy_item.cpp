/*!
  \file        generic_joy_item.cpp
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
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <ros/ros.h>

int button_item = -1;
bool item_before = false;
ros::Publisher item_pub;

////////////////////////////////////////////////////////////////////////////////

void joy_cb(const sensor_msgs::Joy::ConstPtr& joy) {
  int nbuttons = joy->buttons.size();
  // jumps
  bool item_now = (button_item >=0 && nbuttons > button_item
                   && joy->buttons[button_item]);
  if (item_now && !item_before) {
    ROS_INFO("Item button pressed!");
    item_pub.publish(std_msgs::Empty());
  }
  item_before = item_now;
} // end joy_cb();

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "generic_joy_item");
  ros::NodeHandle nh_public, nh_private("~");
  // params
  nh_public.param("button_item", button_item, button_item);
  // subscribers
  ros::Subscriber joy_sub = nh_public.subscribe<sensor_msgs::Joy>("joy", 1,  joy_cb);
  // publishers
  item_pub = nh_public.advertise<std_msgs::Empty>("item", 1);
  ros::spin();
  return 0;
}


