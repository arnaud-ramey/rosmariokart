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
  enum ItemCurse {
    ITEM_BOO             = 0,
    ITEM_GOLDENMUSHROOM  = 1,
    ITEM_LIGHTNING       = 2,
    ITEM_MIRROR          = 3,
    ITEM_MUSHROOM        = 4,
    ITEM_REDSHELL        = 5,
    ITEM_REDSHELL2       = 6,
    ITEM_REDSHELL3       = 7,
    ITEM_STAR            = 8,
    ITEM_TIMEBOMB        = 9,
    ITEM_NONE            = 10,
    ITEM_ROULETTE        = 11,
    CURSE_BOO            = 12,
    CURSE_GOLDENMUSHROOM = 13,
    CURSE_LIGHTNING      = 14,
    CURSE_MIRROR         = 15,
    CURSE_REDSHELL       = 16,
    CURSE_STAR           = 17,
    CURSE_TIMEBOMB       = 18,
  };
  static const unsigned int NITEMS = 10; // ITEM_NONE
  static const unsigned int NITEMSCURSES = 19; // last item + 1

  Rosmariokart() : _nh_private("~") {
    // get params
    _nh_private.param("axis_180turn", _axis_180turn, 4);
    _nh_private.param("axis_90turn", _axis_90turn, 3);
    _nh_private.param("axis_angular", _axis_angular, 2);
    _nh_private.param("axis_linear", _axis_linear, 1);
    _nh_private.param("button_item", _button_item, 3);
    _nh_private.param("scale_angular", scale_angular, 1.0);
    _nh_private.param("scale_linear", scale_linear, 1.0);
    std::string pname;
    _nh_private.param("player1_name", pname, std::string("player1"));
    _player_name.push_back(pname);
    _nh_private.param("player2_name", pname, std::string("player2"));
    _player_name.push_back(pname);
    _nh_private.param("player3_name", pname, std::string(""));
    if (pname.length() > 0)
      _player_name.push_back(pname);
    _nh_private.param("player4_name", pname, std::string(""));
    if (pname.length() > 0)
      _player_name.push_back(pname);
    _nplayers = _player_name.size();

    for (unsigned int i = 0; i < _nplayers; ++i) {
      // create subscribers
      _joy_subs.push_back(_nh_public.subscribe<sensor_msgs::Joy>
                          (_player_name[i] + "/joy", 1, &Rosmariokart::joy1_cb, this));
      // create publishers
      cmd_vel_pubs.push_back(_nh_public.advertise<geometry_msgs::Twist>
                             (_player_name[i] + "/cmd_vel", 1));
      sharp_turn_pubs.push_back(_nh_public.advertise<std_msgs::Float32>
                                (_player_name[i] + "/sharp_turn", 1));
    }

    // alloc data
    _data_path = ros::package::getPath("rosmariokart") + std::string("/data/");
    _sound_path =  _data_path + std::string("sounds/");
    _player_item_roi.push_back(cv::Point(48,150));
    _player_item_roi.push_back(cv::Point(452,150));
    _player_item.resize(_nplayers, ITEM_NONE);
    _item_button_before.resize(_nplayers, false);
    _sharp_turn_before.resize(_nplayers, 0);
    _curse_timer.resize(_nplayers);
    // load Items images
    std::string weappath =  _data_path + std::string("items/");
    _bg = cv::imread(weappath + "screen.png", cv::IMREAD_COLOR);
    // load Items
    unsigned int item_w = 300;
    _item_imgs.resize(NITEMSCURSES, cv::Mat3b(item_w,item_w, cv::Vec3b(0, 0, 255)));
    safe_read_itemcurse(ITEM_BOO, "Boo.png");
    safe_read_itemcurse(ITEM_GOLDENMUSHROOM, "GoldenMushroom.png");
    safe_read_itemcurse(ITEM_LIGHTNING, "Lightning.png");
    safe_read_itemcurse(ITEM_MIRROR, "Mirror.png");
    safe_read_itemcurse(ITEM_MUSHROOM, "Mushroom.png");
    safe_read_itemcurse(ITEM_REDSHELL, "RedShell.png");
    safe_read_itemcurse(ITEM_REDSHELL2, "RedShell2.png");
    safe_read_itemcurse(ITEM_REDSHELL3, "RedShell3.png");
    safe_read_itemcurse(ITEM_STAR, "Star.png");
    safe_read_itemcurse(ITEM_TIMEBOMB, "TimeBomb.png");
    safe_read_itemcurse(CURSE_BOO, "BooCurse.png");
    safe_read_itemcurse(CURSE_GOLDENMUSHROOM, "GoldenMushroomCurse.png");
    safe_read_itemcurse(CURSE_LIGHTNING, "LightningCurse.png");
    safe_read_itemcurse(CURSE_MIRROR, "MirrorCurse.png");
    safe_read_itemcurse(CURSE_REDSHELL, "RedShellCurse.png");
    safe_read_itemcurse(CURSE_STAR, "StarCurse.png");
    safe_read_itemcurse(CURSE_TIMEBOMB, "TimeBombCurse.png");
    // resize items
    for (unsigned int i = 0; i < NITEMSCURSES; ++i)
      cv::resize(_item_imgs[i], _item_imgs[i], cv::Size(item_w, item_w));
  }

  //////////////////////////////////////////////////////////////////////////////

  bool safe_read_itemcurse(unsigned int idx, const std::string & filename) {
    if (idx >= NITEMSCURSES) {
      ROS_WARN("Index '%i' out of range [0, %i]", idx, NITEMSCURSES);
      return false;
    }
    std::string fullfilename = _data_path + std::string("items/") + filename;
    cv::imread(fullfilename , cv::IMREAD_COLOR).copyTo(_item_imgs[idx]);
    if (_item_imgs[idx].empty()) {
      ROS_WARN("Could not load image '%s'", fullfilename.c_str());
      return false;
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool refresh() {
    // check items
    for (unsigned int player_idx = 0; player_idx < _nplayers; ++player_idx) {
      ItemCurse w = _player_item[player_idx];
      double time_curse = _curse_timer[player_idx].getTimeMilliseconds();
      if (w == ITEM_TIMEBOMB && _timebomb.getTimeMilliseconds() > 4935) { // 4.935 seconds
        _player_item[player_idx] = CURSE_TIMEBOMB;
        _curse_timer[player_idx].reset();
      }
      else if (w == ITEM_ROULETTE) {
        if (time_curse > 3000) // roulette timeout
          item_button_cb(player_idx, true);
        else if (_last_roulette_play.getTimeMilliseconds() > 782) { // 0.782 seconds
          play_sound("itemreel.wav");
          _last_roulette_play.reset();
        }
      } // end ITEM_ROULETTE
      // curses
      else if (w == CURSE_BOO && time_curse > 2000) {
        _player_item[player_idx] = ITEM_NONE;
      }
      else if (w == CURSE_GOLDENMUSHROOM && time_curse > 2000) {
        _player_item[player_idx] = ITEM_NONE;
      }
      else if (w == CURSE_LIGHTNING && time_curse > 2000) {
        _player_item[player_idx] = ITEM_NONE;
      }
      else if (w == CURSE_MIRROR && time_curse > 2000) {
        _player_item[player_idx] = ITEM_NONE;
      }
      else if (w == CURSE_REDSHELL && time_curse > 100
               && (rand() % 50 == 0)) { // random end time
        play_sound("cpuspin.wav");
        _player_item[player_idx] = ITEM_NONE;
      }
      else if (w == CURSE_TIMEBOMB && time_curse > 2000) {
        _player_item[player_idx] = ITEM_NONE;
      }
      else if (w == CURSE_STAR && time_curse > 3130) {
        _player_item[player_idx] = ITEM_NONE;
      }
    } // end for player_idx

    // redraw only if needed
    bool need_redraw = (_player_item != _last_drawn_player_item);
    // redraw if there is at least a roulette
    if (std::find(_player_item.begin(), _player_item.end(), ITEM_ROULETTE)
        != _player_item.end())
      need_redraw = true;

    if (need_redraw) {
      //ROS_INFO("Redrawing %g s!", _last_roulette_play.getTimeMilliseconds());
      _bg.copyTo(_final_screen);
      for (unsigned int i = 0; i < _nplayers; ++i) {
        ItemCurse w = _player_item[i];
        unsigned int tlx = _player_item_roi[i].x, tly = _player_item_roi[i].y;
        if (w < NITEMS)
          paste_img(_item_imgs[w], _final_screen, tlx, tly);
        else if (w == ITEM_ROULETTE)
          paste_img(_item_imgs[rand()%NITEMS], _final_screen, tlx, tly);
        else if (w >= CURSE_BOO)
          paste_img(_item_imgs[(int) w], _final_screen, tlx, tly);
      } // end for i
      _last_drawn_player_item = _player_item;
      cv::imshow("rosmariokart", _final_screen);
    } // end if need_redraw
    char c = cv::waitKey(50);
    //ROS_WARN("c:%i", c);
    if (c == 27)
      ros::shutdown();
    else if (c == 'a')
      item_button_cb(0, true);
    else if (c == 'z')
      item_button_cb(1, true);
    else if (c == 'q')
      set_player_roulette(0);
    else if (c == 's')
      set_player_roulette(1);
    else if (c == 81) // left
      sharp_turn_button_cb(-M_PI_2, 0, true);
    else if (c == 83) // right
      sharp_turn_button_cb(M_PI_2, 0, true);
    else if (c == 82) // up
      sharp_turn_button_cb(2 * M_PI, 0, true);
    else if (c == 84) // down
      sharp_turn_button_cb(-M_PI, 0, true);
    return true;
  } // refresh

protected:
  inline ItemCurse random_item() const {
    return (ItemCurse) (rand() % NITEMS);
  }

  //////////////////////////////////////////////////////////////////////////////
  void play_sound(const std::string & filename) {
    std::ostringstream cmd;
    cmd << "aplay --quiet " << _sound_path << filename << " &";
    system(cmd.str().c_str());
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void set_player_roulette(unsigned int player_idx) {
    _player_item[player_idx] = ITEM_ROULETTE;
    _curse_timer[player_idx].reset();
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \player_idx starts at 0
  bool sharp_turn_button_cb(double angle_rad,
                            unsigned int player_idx,
                            bool skip_repetition_check = false) {
    bool retval = false;
    if (fabs(angle_rad) > 1E-2
        && (skip_repetition_check || fabs(angle_rad - _sharp_turn_before[player_idx])>1E-2)) {
      std_msgs::Float32 msg;
      msg.data = angle_rad;
      sharp_turn_pubs[player_idx].publish(msg);
      ROS_WARN("Sharp turn of angle %g rad!", angle_rad);
      retval = true;
    }
    _sharp_turn_before[player_idx] = angle_rad;
    return retval;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \player_idx starts at 0
  bool item_button_cb(unsigned int player_idx,
                      bool skip_repetition_check = false) {
    // check what to do if Item_button
    if (!skip_repetition_check && _item_button_before[player_idx])
      return false;
    ItemCurse w = _player_item[player_idx];
    unsigned target_player_idx = (player_idx == 0 ? 1 : 0);
    ItemCurse wt = _player_item[target_player_idx];

    if (w == ITEM_BOO) { // swap items
      //play_sound("boo.wav");
      play_sound("boosteal.wav");
      if (wt < NITEMS) {
        _player_item[player_idx] = wt;
        _player_item[target_player_idx] = CURSE_BOO;
        _curse_timer[target_player_idx].reset();
      }
      else { // nothing to steal -> punish player!
        _player_item[player_idx] = CURSE_BOO;
        _curse_timer[player_idx].reset();
      }
    }
    else if (w == ITEM_GOLDENMUSHROOM) {
      play_sound("boost.wav");
      _player_item[player_idx] = CURSE_GOLDENMUSHROOM;
      _curse_timer[player_idx].reset();
    }
    else if (w == ITEM_LIGHTNING) {
      play_sound("lightning.wav");
      if (wt != CURSE_STAR) { // do nothing if target has star
        _player_item[target_player_idx] = CURSE_LIGHTNING;
        _curse_timer[target_player_idx].reset();
      }
      _player_item[player_idx] = ITEM_NONE;
    }
    else if (w == ITEM_MIRROR) {
      play_sound("quartz.wav");
      if (wt != CURSE_STAR) { // do nothing if target has star
        _player_item[target_player_idx] = CURSE_MIRROR;
        _curse_timer[target_player_idx].reset();
      }
      _player_item[player_idx] = ITEM_NONE;
    }
    else if (w == ITEM_MUSHROOM) {
      play_sound("boost.wav");
      _player_item[player_idx] = ITEM_NONE;
    }
    else if (w == ITEM_REDSHELL || w == ITEM_REDSHELL2 || w == ITEM_REDSHELL3) {
      play_sound("cputhrow.wav");
      if (wt != CURSE_STAR) { // do nothing if target has star
        _player_item[target_player_idx] = CURSE_REDSHELL;
        _curse_timer[target_player_idx].reset();
      }
      if (w == ITEM_REDSHELL) // decrease red shell counter
        _player_item[player_idx] = ITEM_NONE;
      else if (w == ITEM_REDSHELL2)
        _player_item[player_idx] = ITEM_REDSHELL;
      else if (w == ITEM_REDSHELL3)
        _player_item[player_idx] = ITEM_REDSHELL2;
    }
    else if (w == ITEM_STAR) {
      play_sound("starman.wav");
      _player_item[player_idx] = CURSE_STAR;
      _curse_timer[player_idx].reset();
    }
    else if (w == ITEM_TIMEBOMB) { // passing timebomb
      play_sound("menumove.wav");
      if (wt != CURSE_STAR) { // do nothing if target has star
        _player_item[player_idx] = ITEM_NONE;
        _player_item[target_player_idx] = ITEM_TIMEBOMB;
      }
    }
    else if (w == ITEM_ROULETTE) { // got a new item
      ItemCurse neww = random_item();
      _player_item[player_idx] = neww;
      if (neww == ITEM_TIMEBOMB) { // new TIMEBOMB!
        play_sound("timebomb.wav");
        _timebomb.reset();
      } else
        play_sound("gotitem.wav");
    } // end if ROULETTE
    _item_button_before[player_idx] = true;
    return true;
  } // end item_button_cb()

  //////////////////////////////////////////////////////////////////////////////

  void joy0_cb(const sensor_msgs::Joy::ConstPtr& joy) { joy_cb(joy, 0); }
  void joy1_cb(const sensor_msgs::Joy::ConstPtr& joy) { joy_cb(joy, 1); }

  //! \player_idx starts at 0
  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy,
              unsigned int player_idx) {
    int naxes = joy->axes.size(), nbuttons = joy->buttons.size();
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

    // check sharp turns at 90° or 180°
    double angle = 0;
    if (fabs(joy->axes[_axis_90turn]) > 0.9)
      angle = (joy->axes[_axis_90turn] < 0 ? M_PI_2 : -M_PI_2);
    if (fabs(joy->axes[_axis_180turn]) > 0.9)
      angle = (joy->axes[_axis_180turn] < 0 ? 2 * M_PI : -M_PI);
    bool command_sent = sharp_turn_button_cb(angle, player_idx);

    // check item button
    if (joy->buttons[_button_item])
      item_button_cb(player_idx);

    // cheat: trigger ROULETTE with button 0
    if (_player_item[player_idx] == ITEM_NONE && joy->buttons[0]) {
      set_player_roulette(player_idx);
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
  std::vector<bool> _sharp_turn_before, _item_button_before;
  Timer _last_roulette_play, _timebomb;
  std::vector<Timer> _curse_timer;

  // opencv stuff
  cv_bridge::CvImage bridge;
  cv::Mat3b _bg, _final_screen;
  std::vector<std::string> _player_name;
  unsigned int _nplayers, _w, _h;
  std::vector<cv::Point> _player_item_roi;
  std::vector<ItemCurse> _player_item, _last_drawn_player_item;
  std::vector<cv::Mat3b> _item_imgs;
}; // end class Rosmariokart

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosmariokart");
  srand(time(NULL));
  Rosmariokart mariokart;
  while (ros::ok()) {
    ros::spinOnce();
    mariokart.refresh();
  } // end while (ros::ok())
  return 0;
}
