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
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/package.h>
#include <opencv2/imgproc/imgproc.hpp>

#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   ROS_INFO_THROTTLE(5, __VA_ARGS__)
//#define DEBUG_PRINT(...)   ROS_WARN(__VA_ARGS__)

class Rosmariokart {
public:
  enum Item {
    ITEM_NONE            = 0,
    ITEM_BOO             = 1,
    ITEM_GOLDENMUSHROOM  = 2,
    ITEM_LIGHTNING       = 3,
    ITEM_MIRROR          = 4,
    ITEM_MUSHROOM        = 5,
    ITEM_REDSHELL        = 6,
    ITEM_REDSHELL2       = 7,
    ITEM_REDSHELL3       = 8,
    ITEM_STAR            = 9,
    ITEM_ROULETTE        = 10,
  };
  enum Curse {
    CURSE_NONE               = 0,
    CURSE_BOO                = 1,
    CURSE_GOLDENMUSHROOM     = 2,
    CURSE_LIGHTNING          = 3,
    CURSE_MIRROR             = 4,
    CURSE_MUSHROOM           = 5,
    CURSE_REDSHELL_COMING    = 6,
    CURSE_REDSHELL_HIT       = 7,
    CURSE_STAR               = 8,
    CURSE_TIMEBOMB_COUNTDOWN = 9,
    CURSE_TIMEBOMB_HIT       = 11,
  };
  static const unsigned int NITEMS = 11; // = ITEM_ROULETTE
  static const unsigned int NCURSES = 12; // = CURSE_TIMEBOMB

  Rosmariokart() : _nh_private("~") {
    // gui params
    _nh_private.param("gui_w", _gui_w, 800);
    _nh_private.param("gui_h", _gui_h, 600);
    // player params
    Player p1, p2, p3, p4;
    _nh_private.param("player1_name", p1.name, std::string("player1"));
    _players.push_back(p1);
    _nh_private.param("player2_name", p2.name, std::string("player2"));
    _players.push_back(p2);
    _nh_private.param("player3_name", p3.name, std::string(""));
    if (p3.name.length() > 0)
      _players.push_back(p3);
    _nh_private.param("player4_name", p4.name, std::string(""));
    if (p4.name.length() > 0)
      _players.push_back(p4);
    _nplayers = _players.size();
    // item params
    _curse_timeout.resize(NCURSES, 1);
    _nh_private.param("curse_boo_timeout", _curse_timeout[CURSE_BOO], 2.);
    _nh_private.param("curse_goldenmushroom_timeout", _curse_timeout[CURSE_GOLDENMUSHROOM], 2.);
    _nh_private.param("curse_lightning_timeout", _curse_timeout[CURSE_LIGHTNING], 2.);
    _nh_private.param("curse_mirror_timeout", _curse_timeout[CURSE_MIRROR], 2.);
    _nh_private.param("curse_mushroom_timeout", _curse_timeout[CURSE_MUSHROOM], 2.);
    _nh_private.param("curse_redshell_coming_timeout", _curse_timeout[CURSE_REDSHELL_COMING], 2.);
    _nh_private.param("curse_redshell_hit_timeout", _curse_timeout[CURSE_REDSHELL_HIT], 2.);
    _nh_private.param("curse_star_timeout", _curse_timeout[CURSE_STAR], 3.130);
    _nh_private.param("curse_timebomb_hit_timeout", _curse_timeout[CURSE_TIMEBOMB_HIT], 2.);

    // joy params
    _nh_private.param("axis_180turn", _axis_180turn, 4);
    _nh_private.param("axis_90turn", _axis_90turn, 3);
    _nh_private.param("axis_angular", _axis_angular, 2);
    _nh_private.param("axis_linear", _axis_linear, 1);
    _nh_private.param("button_item", _button_item, 3);

    for (unsigned int i = 0; i < _nplayers; ++i) {
      // params
      _nh_public.param(_players[i].name + "/scale_angular", _players[i].scale_angular, 1.0);
      _nh_public.param(_players[i].name + "/scale_linear", _players[i].scale_linear, 1.0);
      // create subscribers - pass i
      // http://ros-users.122217.n3.nabble.com/How-to-identify-the-subscriber-or-the-topic-name-related-to-a-callback-td2391327.html
      _players[i].joy_sub = _nh_public.subscribe<sensor_msgs::Joy>
          (_players[i].name + "/joy", 1, boost::bind(&Rosmariokart::joy_cb, this, _1, i));
      // create publishers
      _players[i].cmd_vel_pub  = _nh_public.advertise<geometry_msgs::Twist>
          (_players[i].name + "/cmd_vel", 1);
      _players[i].animation_pub  = _nh_public.advertise<std_msgs::String>
          (_players[i].name + "/animation", 1);
      _players[i].sharp_turn_pub = _nh_public.advertise<std_msgs::Float32>
          (_players[i].name + "/sharp_turn", 1);
    }

    // alloc data
    _data_path = ros::package::getPath("rosmariokart") + std::string("/data/");
    _sound_path =  _data_path + std::string("sounds/");
    // configure GUI
    _gui_bg.create(_gui_h, _gui_w);
    _gui_bg.setTo(cv::Vec3b(0,0,0));
    if (_nplayers == 2) {
      _item_w = std::min(_gui_w / 2, _gui_h);
      int paddingh = (_gui_h - _item_w) / 2;
      _players[0].item_roi  = cv::Point(0,paddingh);
      _players[1].item_roi  = cv::Point(_gui_w/2, paddingh);
    }
    else if (_nplayers == 3) {
      ROS_FATAL("Not implemented");
      ros::shutdown();
    }
    else if (_nplayers == 4) {
      ROS_FATAL("Not implemented");
      ros::shutdown();
    }
    // load Items
    _item_imgs.resize(NITEMS, cv::Mat3b(_item_w,_item_w, cv::Vec3b(0, 0, 255)));
    imread_vector(_item_imgs, ITEM_BOO, "Boo.png");
    imread_vector(_item_imgs, ITEM_GOLDENMUSHROOM, "GoldenMushroom.png");
    imread_vector(_item_imgs, ITEM_LIGHTNING, "Lightning.png");
    imread_vector(_item_imgs, ITEM_MIRROR, "Mirror.png");
    imread_vector(_item_imgs, ITEM_MUSHROOM, "Mushroom.png");
    imread_vector(_item_imgs, ITEM_REDSHELL, "RedShell.png");
    imread_vector(_item_imgs, ITEM_REDSHELL2, "RedShell2.png");
    imread_vector(_item_imgs, ITEM_REDSHELL3, "RedShell3.png");
    imread_vector(_item_imgs, ITEM_STAR, "Star.png");
    // load curses
    _curse_imgs.resize(NCURSES, cv::Mat3b(_item_w, _item_w, cv::Vec3b(0, 0, 255)));
    imread_vector(_curse_imgs, CURSE_BOO, "BooCurse.png");
    imread_vector(_curse_imgs, CURSE_GOLDENMUSHROOM, "GoldenMushroomCurse.png");
    imread_vector(_curse_imgs, CURSE_LIGHTNING, "LightningCurse.png");
    imread_vector(_curse_imgs, CURSE_MIRROR, "MirrorCurse.png");
    imread_vector(_curse_imgs, CURSE_MUSHROOM, "MushroomCurse.png");
    imread_vector(_curse_imgs, CURSE_REDSHELL_HIT, "RedShellCurse.png");
    imread_vector(_curse_imgs, CURSE_REDSHELL_COMING, "RedShellComing.png");
    imread_vector(_curse_imgs, CURSE_STAR, "StarCurse.png");
    imread_vector(_curse_imgs, CURSE_TIMEBOMB_COUNTDOWN, "TimeBombCountdown.png");
    imread_vector(_curse_imgs, CURSE_TIMEBOMB_HIT, "TimeBombCurse.png");
    // resize items
    for (unsigned int i = 0; i < NITEMS; ++i)
      cv::resize(_item_imgs[i], _item_imgs[i], cv::Size(_item_w, _item_w));
    for (unsigned int i = 0; i < NCURSES; ++i)
      cv::resize(_curse_imgs[i], _curse_imgs[i], cv::Size(_item_w, _item_w));
  }

  //////////////////////////////////////////////////////////////////////////////

  bool imread_vector(std::vector<cv::Mat3b> & v,
                     unsigned int idx, const std::string & filename) {
    if (idx >= v.size()) {
      ROS_WARN("Index '%i' out of range [0, %i]", idx, v.size());
      return false;
    }
    std::string fullfilename = _data_path + std::string("items/") + filename;
    cv::imread(fullfilename , cv::IMREAD_COLOR).copyTo(v[idx]);
    if (v[idx].empty()) {
      ROS_WARN("Could not load image '%s'", fullfilename.c_str());
      return false;
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool refresh() {
    // check items
    for (unsigned int player_idx = 0; player_idx < _nplayers; ++player_idx) {
      Player* p = &(_players[player_idx]);

      // check items
      if (p->item == ITEM_ROULETTE) {
        if (p->roulette_timer.getTimeSeconds() > 3) // roulette timeout
          item_button_cb(player_idx, true);
        // rewind sound
        else if (_last_roulette_play.getTimeSeconds() > .782) { // 0.782 seconds
          play_sound("itemreel.wav");
          _last_roulette_play.reset();
        }
      } // end ITEM_ROULETTE

      // clean curses if needed
      double time_curse = p->curse_timer.getTimeSeconds();
      if (p->curse == CURSE_REDSHELL_COMING
          && time_curse > .1
          && (time_curse > _curse_timeout[CURSE_REDSHELL_COMING]
              || rand() % 50 == 0)) { // random end time
        play_sound("cpuspin.wav");
        p->play_animation((rand()%2 ? "hit" : "hit2"));
        p->play_animation_caster(player_idx, _players, "mock");
        p->receive_curse(CURSE_REDSHELL_HIT, p->curse_caster_idx);
      }
      else if (p->curse == CURSE_TIMEBOMB_COUNTDOWN
               && _timebomb.getTimeSeconds() > 4.935) { // 4.935 seconds
        p->play_animation((rand()%2 ? "hit" : "hit2"));
        p->play_animation_caster(player_idx, _players, "mock");
        p->receive_curse(CURSE_TIMEBOMB_HIT, p->curse_caster_idx);
      }
      else if ((p->curse != CURSE_NONE) // whatever curse
               && time_curse > _curse_timeout[p->curse]) {
        p->curse = CURSE_NONE;
      }
    } // end for player_idx

    bool need_imshow = false;
    if (_gui_final.empty()) { // first time display
      _gui_bg.copyTo(_gui_final);
      need_imshow = true;
    }

    for (unsigned int i = 0; i < _nplayers; ++i) {
      Player* p = &(_players[i]);
      // redraw only if needed
      bool need_redraw = (p->item != p->last_drawn_item
          || p->curse != p->last_drawn_curse);
      // redraw if user has a roulette
      if (p->item == ITEM_ROULETTE)
        need_redraw = true;
      if (need_redraw)
        need_imshow = true;

      if (need_redraw) {
        unsigned int tlx = p->item_roi.x, tly = p->item_roi.y;
        if (p->curse != CURSE_NONE)
          paste_img(_curse_imgs[(int) p->curse], _gui_final, tlx, tly);
        else if (p->item == ITEM_ROULETTE)
          paste_img(_item_imgs[random_item()], _gui_final, tlx, tly);
        else if (p->item != ITEM_NONE)
          paste_img(_item_imgs[p->item], _gui_final, tlx, tly);
        else {// (CURSE_NONE && ITEM_NONE)
          cv::Rect roi(tlx, tly, _item_w, _item_w);
          cv::Mat3b item_roi(_gui_final(roi));
          _gui_bg(roi).copyTo(item_roi);
        }
        p->last_drawn_item = p->item;
        p->last_drawn_curse = p->curse;
      } // end if need_redraw
    } // end for i

    if (need_imshow) {
      DEBUG_PRINT("imshow()-%g s!", _last_roulette_play.getTimeSeconds());
      cv::imshow("rosmariokart", _gui_final);
    }
    char c = cv::waitKey(50);
    DEBUG_PRINT("c:%i", c);
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
  } // end refresh()

protected:
  static bool is_real_item(Item i) {
    return (i != ITEM_NONE && i != ITEM_ROULETTE);
  }

  //////////////////////////////////////////////////////////////////////////////

  Item random_item() const {
    return ITEM_LIGHTNING; // debug test
    Item i = (Item) (rand() % NITEMS);
    return (is_real_item(i) ? i : random_item());
  }

  //////////////////////////////////////////////////////////////////////////////

  bool play_sound(const std::string & filename) const {
    std::ostringstream cmd;
    cmd << "aplay --quiet " << _sound_path << filename << " &";
    return (system(cmd.str().c_str()) == 0);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool set_player_roulette(unsigned int player_idx) {
    DEBUG_PRINT("set_player_roulette(%i)", player_idx);
    Player* p = &(_players[player_idx]);
    if (p->item != ITEM_NONE)
      return false;
    if (p->curse != CURSE_NONE)
      return false;
    p->item = ITEM_ROULETTE;
    p->roulette_timer.reset();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \player_idx starts at 0
  bool sharp_turn_button_cb(double angle_rad,
                            unsigned int player_idx,
                            bool skip_repetition_check = false) {
    Player* p = &(_players[player_idx]);
    bool retval = false;
    if (fabs(angle_rad) > 1E-2
        && (skip_repetition_check || fabs(angle_rad - p->sharp_turn_before)>1E-2)) {
      std_msgs::Float32 msg;
      msg.data = angle_rad;
      p->sharp_turn_pub.publish(msg);
      DEBUG_PRINT("Player %i: sharp turn of angle %g rad!", player_idx, angle_rad);
      retval = true;
    }
    p->sharp_turn_before = angle_rad;
    return retval;
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \player_idx starts at 0
  bool item_button_cb(unsigned int player_idx,
                      bool skip_repetition_check = false) {
    DEBUG_PRINT("item_button_cb(%i, %i)", player_idx, skip_repetition_check);
    unsigned target_player_idx = (player_idx == 0 ? 1 : 0);
    Player* p = &(_players[player_idx]), *target = &(_players[target_player_idx]);
    // check what to do if Item_button
    if (!skip_repetition_check && p->item_button_before)
      return false;
    Item pi = p->item;
    Curse pc = p->curse;
    Item targeti = target->item;
    Curse targetc = target->curse;

    // check curses
    if (pc == CURSE_TIMEBOMB_COUNTDOWN) { // passing timebomb
      if (targetc != CURSE_STAR) { // do nothing if target has star
        play_sound("menumove.wav");
        p->curse = CURSE_NONE;
        target->receive_curse(CURSE_TIMEBOMB_COUNTDOWN, player_idx);
      }
    } // end CURSE_TIMEBOMB_COUNTDOWN

    // check items
    if (pi == ITEM_BOO) { // swap items
      //play_sound("boo.wav");
      play_sound("boosteal.wav");
      p->item = ITEM_NONE;
      if (is_real_item(targeti)) {
        p->item = targeti;
        target->item = ITEM_NONE;
        target->receive_curse(CURSE_BOO, player_idx);
      }
      else { // nothing to steal -> punish player!
        p->receive_curse(CURSE_BOO, player_idx);
      }
    }
    else if (pi == ITEM_GOLDENMUSHROOM) {
      play_sound("boost.wav");
      p->item = ITEM_NONE;
      p->receive_curse(CURSE_GOLDENMUSHROOM, player_idx);
    }
    else if (pi == ITEM_LIGHTNING) {
      play_sound("lightning.wav");
      p->item = ITEM_NONE;
      if (targetc != CURSE_STAR) { // do nothing if target has star
        target->receive_curse(CURSE_LIGHTNING, player_idx);
        p->play_animation("mock");
      }
    }
    else if (pi == ITEM_MIRROR) {
      play_sound("quartz.wav");
      p->item = ITEM_NONE;
      if (targetc != CURSE_STAR) { // do nothing if target has star
        target->receive_curse(CURSE_MIRROR, player_idx);
        p->play_animation("mock");
      }
    }
    else if (pi == ITEM_MUSHROOM) {
      play_sound("boost.wav");
      p->item = ITEM_NONE;
      p->curse = CURSE_MUSHROOM;
      p->curse_timer.reset();
    }
    else if (pi == ITEM_REDSHELL || pi == ITEM_REDSHELL2 || pi == ITEM_REDSHELL3) {
      play_sound("cputhrow.wav");
      if (targetc != CURSE_STAR) // do nothing if target has star
        target->receive_curse(CURSE_REDSHELL_COMING, player_idx);
      if (pi == ITEM_REDSHELL) // decrease red shell counter
        p->item = ITEM_NONE;
      else if (pi == ITEM_REDSHELL2)
        p->item = ITEM_REDSHELL;
      else if (pi == ITEM_REDSHELL3)
        p->item = ITEM_REDSHELL2;
    }
    else if (pi == ITEM_STAR) {
      play_sound("starman.wav");
      p->item = ITEM_NONE;
      p->receive_curse(CURSE_STAR, player_idx);
    }
    else if (pi == ITEM_ROULETTE) { // got a new item
      if (pc == CURSE_NONE && rand() % 20 == 0) { // new TIMEBOMB!
        play_sound("timebomb.wav");
        p->item = ITEM_NONE;
        p->receive_curse(CURSE_TIMEBOMB_COUNTDOWN, player_idx);
        _timebomb.reset();
      } else {
        Item neww = random_item();
        p->item = neww;
        play_sound("gotitem.wav");
      }
    } // end if ROULETTE

    p->item_button_before = true;
    return true;
  } // end item_button_cb()

  //////////////////////////////////////////////////////////////////////////////

  //! \player_idx starts at 0
  void set_speed(unsigned int player_idx,
                 double v,
                 double w) {
    DEBUG_PRINT("set_speed(%i, %g, %g)", player_idx, v, w);
    Player* p = &(_players[player_idx]);
    geometry_msgs::Twist vel;
    vel.linear.x = v;
    vel.angular.z = w;
    // now check for curses
    Curse c = p->curse;
    if (c == CURSE_GOLDENMUSHROOM) {
      vel.linear.x *= 5; // 500% faster
    }
    else if (c == CURSE_LIGHTNING) {
      ROS_WARN("%i:CURSE_LIGHTNING!", player_idx);
      vel.linear.x *= .3; // half speed
      vel.angular.z *= .3; // half speed
    }
    else if (c == CURSE_MIRROR) { // inverted commands
      vel.linear.x *= -1;
      vel.angular.z *= -1;
    }
    if (c == CURSE_MUSHROOM) {
      vel.linear.x *= 5; // 500% faster
    }
    else if (c == CURSE_REDSHELL_HIT || c == CURSE_TIMEBOMB_HIT) {
      vel = geometry_msgs::Twist(); // no move
    }
    else if (c == CURSE_STAR) {
      vel.linear.x *= 1.2; // 20% faster
    }

    p->cmd_vel_pub.publish(vel);
  } // end set_speed()

  //////////////////////////////////////////////////////////////////////////////

  //! \player_idx starts at 0
  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy,
              unsigned int player_idx) {
    DEBUG_PRINT("joy_cb(%i)", player_idx);
    int naxes = joy->axes.size(), nbuttons = joy->buttons.size();
    if (naxes < _axis_90turn
        || naxes < _axis_180turn
        || naxes < _axis_linear
        || naxes < _axis_angular) {
      ROS_WARN_THROTTLE(1, "Only %i axes on joystick #%i!", naxes, player_idx);
      return;
    }
    if (nbuttons < _button_item) {
      ROS_WARN_THROTTLE(1, "Only %i buttons on joystick #%i!", nbuttons, player_idx);
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
    Player* p = &(_players[player_idx]);
    if (joy->buttons[_button_item])
      item_button_cb(player_idx);
    else {
      p->item_button_before = false;
    }

    // cheat: trigger ROULETTE with button 0
    if (joy->buttons[0]) {
      set_player_roulette(player_idx);
    }

    // if no command was sent till here: move robot with directions of axes
    if (command_sent)
      return;
    set_speed(player_idx,
              joy->axes[_axis_linear] * p->scale_linear,
              joy->axes[_axis_angular] * p->scale_angular);
  } // end joy_cb();

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  struct Player {
    Player() { // ctor
      item = last_drawn_item = ITEM_NONE;
      curse = last_drawn_curse = CURSE_NONE;
      scale_linear = scale_angular = 1;
      item_button_before = sharp_turn_before = false;
    }
    void receive_curse(Curse c, unsigned int curse_caster_) {
      curse = c;
      curse_timer.reset();
      curse_caster_idx = curse_caster_;
    }
    void play_animation(const std::string & s) {
      std_msgs::String anim;
      anim.data = s;
      animation_pub.publish(anim);
    }
    void play_animation_caster(unsigned int self_idx,
                               std::vector<Player> & players,
                               const std::string & s) {
      if (self_idx == curse_caster_idx)
        return;
      players[curse_caster_idx].play_animation(s);
    }

    std::string name;
    cv::Point item_roi;
    ros::Subscriber joy_sub;
    ros::Publisher cmd_vel_pub, sharp_turn_pub, animation_pub;
    Item item, last_drawn_item;
    Curse curse, last_drawn_curse;
    bool sharp_turn_before, item_button_before;
    double scale_angular, scale_linear;
    Timer curse_timer, roulette_timer;
    unsigned int curse_caster_idx;
  }; // end struct Player

  ros::NodeHandle _nh_public, _nh_private;
  int _axis_linear, _axis_angular, _axis_90turn, _axis_180turn, _button_item;
  Timer _last_roulette_play, _timebomb;

  // opencv stuff
  int _gui_w, _gui_h;
  cv::Mat3b _gui_bg, _gui_final;
  unsigned int _nplayers;
  std::vector<Player> _players;

  // items
  std::string _data_path, _sound_path;
  std::vector<cv::Mat3b> _item_imgs, _curse_imgs;
  unsigned int _item_w; // pixels
  std::vector<double> _curse_timeout;
}; // end class Rosmariokart

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosmariokart");
  srand(time(NULL));
  Rosmariokart mariokart;
  //ros::AsyncSpinner spinner(0);
  //spinner.start();
  ros::Rate rate(50);
  while (ros::ok()) {
    mariokart.refresh();
    ros::spinOnce();
    rate.sleep();
  } // end while (ros::ok())
  return 0;
}
