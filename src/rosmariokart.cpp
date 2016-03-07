/*!
  \file        rosmariokart.cpp
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

\todo Description of the file

\section Parameters
 */
// third parties
#include "rosmariokart/rosmariokart.h"
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>

class Rosmariokart {
public:
  enum Item {
    BOO             = 0,
    GOLDENMUSHROOM  = 1,
    LIGHTNING       = 2,
    MIRROR          = 3,
    MUSHROOM        = 4,
    REDSHELL        = 5,
    REDSHELL2       = 6,
    REDSHELL3       = 7,
    STAR            = 8,
    TIMEBOMB        = 9,
    NONE            = 10,
    ROULETTE         = 11,
  };
  static const unsigned int NITEMS = 10;

  Rosmariokart() : _nh_private("~") {
    // get params
    _nh_private.param("axis_180turn", _axis_180turn, 4);
    _nh_private.param("axis_90turn", _axis_90turn, 3);
    _nh_private.param("axis_angular", _axis_angular, 2);
    _nh_private.param("axis_linear", _axis_linear, 1);
    _nh_private.param("button_item", _button_item, 3);
    _nh_private.param("scale_angular", scale_angular, 1.0);
    _nh_private.param("scale_linear", scale_linear, 1.0);
    _nplayers = 2;

    // create subscribers
    _joy_subs.push_back(_nh_public.subscribe<sensor_msgs::Joy>
                        ("joy0", 1, &Rosmariokart::joy0_cb, this));
    _joy_subs.push_back(_nh_public.subscribe<sensor_msgs::Joy>
                        ("joy1", 1, &Rosmariokart::joy1_cb, this));
    // create publishers
    for (unsigned int i = 0; i < _nplayers; ++i) {
      cmd_vel_pubs.push_back(_nh_public.advertise<geometry_msgs::Twist>
                             ("cmd_vel" + cast2string(i), 1));
      sharp_turn_pubs.push_back(_nh_public.advertise<std_msgs::Float32>
                                ("sharp_turn" + cast2string(i), 1));
    }

    // alloc data
    _data_path = ros::package::getPath("rosmariokart") + std::string("/data/");
    _sound_path =  _data_path + std::string("sounds/");
    _player_item_roi.push_back(cv::Point(48,150));
    _player_item_roi.push_back(cv::Point(452,150));
    _player_item.resize(_nplayers, NONE);
    _item_button_before.resize(_nplayers, false);
    _axis_90before.resize(_nplayers, false);
    _axis_180before.resize(_nplayers, false);
    // load Items images
    std::string weappath =  _data_path + std::string("items/");
    _bg = cv::imread(weappath + "screen.png", cv::IMREAD_COLOR);
    // load Items
    _item_imgs.push_back(cv::imread(weappath + "Boo.png", cv::IMREAD_COLOR));
    _item_imgs.push_back(cv::imread(weappath + "GoldenMushroom.png", cv::IMREAD_COLOR));
    _item_imgs.push_back(cv::imread(weappath + "Lightning.png", cv::IMREAD_COLOR));
    _item_imgs.push_back(cv::imread(weappath + "Mirror.png", cv::IMREAD_COLOR));
    _item_imgs.push_back(cv::imread(weappath + "Mushroom.png", cv::IMREAD_COLOR));
    _item_imgs.push_back(cv::imread(weappath + "RedShell.png", cv::IMREAD_COLOR));
    _item_imgs.push_back(cv::imread(weappath + "RedShell2.png", cv::IMREAD_COLOR));
    _item_imgs.push_back(cv::imread(weappath + "RedShell3.png", cv::IMREAD_COLOR));
    _item_imgs.push_back(cv::imread(weappath + "Star.png", cv::IMREAD_COLOR));
    _item_imgs.push_back(cv::imread(weappath + "TimeBomb.png", cv::IMREAD_COLOR));
    assert(_item_imgs.size() == NITEMS);
    // resize Items
    unsigned int Item_w = 300;
    for (unsigned int i = 0; i < NITEMS; ++i)
      cv::resize(_item_imgs[i], _item_imgs[i], cv::Size(Item_w, Item_w));
  }

  //////////////////////////////////////////////////////////////////////////////

  bool refresh() {
    // redraw only if needed
    bool need_redraw = (_player_item != _last_drawn_player_item);
    // redraw if there is at least a roulette
    if (std::find(_player_item.begin(), _player_item.end(), ROULETTE)
        != _player_item.end())
      need_redraw = true;

    if (need_redraw) {
      ROS_INFO("Redrawing %g s!", _last_roulette_play.getTimeMilliseconds());
      _bg.copyTo(_final_screen);
      for (unsigned int i = 0; i < _nplayers; ++i) {
        Item w = _player_item[i];
        unsigned int tlx = _player_item_roi[i].x, tly = _player_item_roi[i].y;
        if (w < NITEMS)
          paste_img(_item_imgs[w], _final_screen, tlx, tly);
        else if (w == ROULETTE)
          paste_img(_item_imgs[rand()%NITEMS], _final_screen, tlx, tly);
      } // end for i
      _last_drawn_player_item = _player_item;
      cv::imshow("rosmariokart", _final_screen);
    } // end if need_redraw
    char c = cv::waitKey(50);
    if (c == 27)
      ros::shutdown();
    return true;
  } // refresh

protected:
  //////////////////////////////////////////////////////////////////////////////
  void play_sound(const std::string & filename) {
    std::ostringstream cmd;
    cmd << "aplay --quiet " << _sound_path << filename << " &";
    system(cmd.str().c_str());
  }

  void joy0_cb(const sensor_msgs::Joy::ConstPtr& joy) { joy_cb(joy, 0); }
  void joy1_cb(const sensor_msgs::Joy::ConstPtr& joy) { joy_cb(joy, 1); }

  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy,
              unsigned int player_idx) {
    int naxes = joy->axes.size(), nbuttons = joy->buttons.size();
    unsigned target_player_idx = (player_idx == 0 ? 1 : 0);
    if (naxes < _axis_90turn
        || naxes < _axis_180turn
        || naxes < _axis_linear
        || naxes < _axis_angular) {
      ROS_WARN("Only %i axes on joystick #%i!", naxes, player_idx);
      return;
    }
    if (nbuttons < _button_item) {
      ROS_WARN("Only %i buttons on joystick #%i!", nbuttons, player_idx);
      return;
    }

    // check sharp turns
    bool command_sent = false;
    // sharp turns at 90°
    bool axis_90now = fabs(joy->axes[_axis_90turn]) > 0.9;
    if (axis_90now && !_axis_90before[player_idx]) {
      std_msgs::Float32 msg;
      msg.data = (joy->axes[_axis_90turn] < 0 ? M_PI_2 : -M_PI_2);
      sharp_turn_pubs[player_idx].publish(msg);
      command_sent = true;
    }
    _axis_90before[player_idx] = axis_90now;
    // sharp turns at 180°
    bool axis_180now = fabs(joy->axes[_axis_180turn]) > 0.9;
    if (axis_180now && !_axis_180before[player_idx]) {
      std_msgs::Float32 msg;
      msg.data = (joy->axes[_axis_180turn] > 0 ? 2 * M_PI : -M_PI);
      sharp_turn_pubs[player_idx].publish(msg);
      command_sent = true;
    }
    _axis_180before[player_idx] = axis_180now;

    Item w = _player_item[player_idx];
    if (w == ROULETTE && _last_roulette_play.getTimeMilliseconds() > 782) { // 0.782 seconds
      play_sound("itemreel.wav");
      _last_roulette_play.reset();
    }
    if (w == TIMEBOMB && _timebomb.getTimeMilliseconds() > 4935) { // 4.935 seconds
      _player_item[player_idx] = NONE;
    }

    // check what to do if Item_button
    if (joy->buttons[_button_item] && !_item_button_before[player_idx]) {
      if (w == BOO) { // swap items
        //play_sound("boo.wav");
        play_sound("boosteal.wav");
        _player_item[player_idx] = _player_item[target_player_idx];
        _player_item[target_player_idx] = NONE;
      }
      else if (w == GOLDENMUSHROOM) {
        play_sound("boost.wav");
        _player_item[player_idx] = NONE;
      }
      else if (w == LIGHTNING) {
        play_sound("lightning.wav");
        _player_item[player_idx] = NONE;
      }
      else if (w == MIRROR) {
        play_sound("quartz.wav");
        _player_item[player_idx] = NONE;
      }
      else if (w == MUSHROOM) {
        play_sound("boost.wav");
        _player_item[player_idx] = NONE;
      }
      else if (w == REDSHELL) {
        play_sound("cputhrow.wav");
        _player_item[player_idx] = NONE;
      }
      else if (w == REDSHELL2) {
        play_sound("cputhrow.wav");
        _player_item[player_idx] = REDSHELL;
      }
      else if (w == REDSHELL3) {
        play_sound("cputhrow.wav");
        _player_item[player_idx] = REDSHELL2;
      }
      else if (w == STAR) {
        play_sound("starman.wav");
        _player_item[player_idx] = NONE;
      }
      else if (w == TIMEBOMB) { // passing timebomb
        play_sound("menumove.wav");
        _player_item[player_idx] = NONE;
        _player_item[target_player_idx] = TIMEBOMB;
      }
      else if (w == ROULETTE) { // got a new item
        Item neww = (Item) (rand() % NITEMS);
        _player_item[player_idx] = neww;
        if (neww == TIMEBOMB) { // new TIMEBOMB!
          play_sound("timebomb.wav");
          _timebomb.reset();
        } else
          play_sound("gotitem.wav");
      } // end if ROULETTE
    } // end if (joy->buttons[_button_Item])
    _item_button_before[player_idx] = joy->buttons[_button_item];

    if (w == NONE && joy->buttons[0]) {
      _player_item[player_idx] = ROULETTE;
    }

    // if no command was sent till here: move robot with directions of axes
    if (command_sent)
      return;
    geometry_msgs::Twist vel;
    vel.linear.x = (joy->axes[_axis_linear] * scale_linear);
    vel.angular.z = (joy->axes[_axis_angular] * scale_angular);
    cmd_vel_pubs[player_idx].publish(vel);
  } // end joy_cb();

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  ros::NodeHandle _nh_public, _nh_private;
  std::string _data_path, _sound_path;
  std::vector<ros::Subscriber> _joy_subs;
  std::vector<ros::Publisher> cmd_vel_pubs, sharp_turn_pubs;
  int _axis_linear, _axis_angular, _axis_90turn, _axis_180turn, _button_item;
  double scale_linear, scale_angular;
  std::vector<bool> _axis_90before, _axis_180before, _item_button_before;
  Timer _last_roulette_play, _timebomb;

  // opencv stuff
  cv_bridge::CvImage bridge;
  cv::Mat3b _bg, _final_screen;
  unsigned int _nplayers, _w, _h;
  std::vector<cv::Point> _player_item_roi;
  std::vector<Item> _player_item, _last_drawn_player_item;
  std::vector<cv::Mat3b> _item_imgs;
}; // end class Rosmariokart

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosmariokart");
  Rosmariokart mariokart;
  while (ros::ok()) {
    ros::spinOnce();
    mariokart.refresh();
  } // end while (ros::ok())
  return 0;
}
