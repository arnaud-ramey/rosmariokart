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
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <ros/package.h>

class Rosmariokart {
public:
  Rosmariokart() : _nh_private("~") {
    // gui params
    _nh_private.param("gui_w", _gui_w, 800);
    _nh_private.param("gui_h", _gui_h, 600);
    _nh_private.param("min_time_roulette", _min_time_roulette, 10.);
    _nh_private.param("timebomb_likelihood", _timebomb_likelihood, 0.03); // 3%
    _nh_private.param("race_duration", _race_duration, 80.); // seconds = 1 min 20
    // item params
    _curse_timeout.resize(NCURSES, 10);
    _nh_private.param("curse_boo_timeout", _curse_timeout[CURSE_BOO], 3.);
    _nh_private.param("curse_dud_start_timeout", _curse_timeout[CURSE_DUD_START], 5.);
    _nh_private.param("curse_goldenmushroom_timeout", _curse_timeout[CURSE_GOLDENMUSHROOM], 3.);
    _nh_private.param("curse_lightning_timeout", _curse_timeout[CURSE_LIGHTNING], 5.);
    _nh_private.param("curse_mirror_timeout", _curse_timeout[CURSE_MIRROR], 5.);
    _nh_private.param("curse_mushroom_timeout", _curse_timeout[CURSE_MUSHROOM], 3.);
    _nh_private.param("curse_redshell_coming_timeout", _curse_timeout[CURSE_REDSHELL_COMING], 3.);
    _nh_private.param("curse_redshell_hit_timeout", _curse_timeout[CURSE_REDSHELL_HIT], 3.);
    _nh_private.param("curse_rocket_start_timeout", _curse_timeout[CURSE_ROCKET_START], 5.);
    _nh_private.param("curse_star_timeout", _curse_timeout[CURSE_STAR], 3.130);
    _nh_private.param("curse_timebomb_hit_timeout", _curse_timeout[CURSE_TIMEBOMB_HIT], 3.);
    // joy params
    _nh_private.param("axis_180turn", _axis_180turn, 4);
    _nh_private.param("axis_90turn", _axis_90turn, 3);
    _nh_private.param("axis_angular", _axis_angular, 2);
    _nh_private.param("axis_linear", _axis_linear, 1);
    _nh_private.param("button_item", _button_item, 3);
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

    for (unsigned int i = 0; i < _nplayers; ++i) {
      Player* p = &(_players[i]);
      // params
      _nh_public.param(p->name + "/scale_angular", p->scale_angular, 1.0);
      _nh_public.param(p->name + "/scale_linear", p->scale_linear, 1.0);
      // create subscribers - pass i
      // http://ros-users.122217.n3.nabble.com/How-to-identify-the-subscriber-or-the-topic-name-related-to-a-callback-td2391327.html
      p->joy_sub = _nh_public.subscribe<sensor_msgs::Joy>
          (p->name + "/joy", 1, boost::bind(&Rosmariokart::joy_cb, this, _1, i));
      // create publishers
      p->cmd_vel_pub  = _nh_public.advertise<geometry_msgs::Twist>
          (p->name + "/cmd_vel", 1);
      p->animation_pub  = _nh_public.advertise<std_msgs::String>
          (p->name + "/animation", 1);
      p->sharp_turn_pub = _nh_public.advertise<std_msgs::Float32>
          (p->name + "/sharp_turn", 1);
    }

    // alloc data
    _data_path = ros::package::getPath("rosmariokart") + std::string("/data/");
    _sound_path =  _data_path + std::string("sounds/");
    // configure GUI
    if (_nplayers == 2) {
      _item_w = std::min(_gui_w / 2, _gui_h);
      int paddingw = (_gui_w - 2 * _item_w) / 2;
      int paddingh = (_gui_h - _item_w) / 2;
      _players[0].item_roi  = cv::Point(paddingw, paddingh);
      _players[1].item_roi  = cv::Point(_gui_w/2, paddingh);
    }
    else if (_nplayers == 3 || _nplayers == 4) {
      _item_w = std::min(_gui_w / 2, _gui_h / 2);
      int paddingw = (_gui_w - 2 * _item_w) / 2;
      int paddingh = (_gui_h - 2 * _item_w) / 2;
      _players[0].item_roi  = cv::Point(paddingw, paddingh);
      _players[1].item_roi  = cv::Point(_gui_w/2, paddingh);
      _players[2].item_roi  = cv::Point(paddingw, _gui_h/2);
      if (_nplayers == 4)
        _players[3].item_roi  = cv::Point(_gui_w/2, _gui_h/2);
    }
    _gui_bg.create(_gui_h, _gui_w);
    _gui_bg.setTo(cv::Vec3b(0,0,0));
    // put robot background
    for (unsigned int i = 0; i < _nplayers; ++i) {
      Player* p = &(_players[i]);
      std::string imgname = "random_robot.png";
      if (p->name.find("mip") != std::string::npos)
        imgname = "white_mip_black_bg.png";
      else if (p->name.find("robot_") != std::string::npos)
        imgname = "stage_black_bg.png";
      else if (p->name.find("sumo") != std::string::npos)
        imgname = "white_sumo_black_bg.png";
      std::string fullfilename = _data_path + std::string("robots/") + imgname;
      cv:: Mat3b m = cv::imread(fullfilename , cv::IMREAD_COLOR);
      if (m.empty())
        continue;
      cv::resize(m, m, cv::Size(_item_w , _item_w));
      m *= .5; // lower brightness
      paste_img(m, _gui_bg, p->item_roi.x, p->item_roi.y);
    } // end for i
    _gui_bg.copyTo(_gui_final);

    // load Items
    _item_imgs.resize(NITEMS, cv::Mat3b(_item_w,_item_w, cv::Vec3b(0, 0, 255)));
    imread_vector(_item_imgs, ITEM_BOO, _data_path + "items/Boo.png", _item_w);
    imread_vector(_item_imgs, ITEM_GOLDENMUSHROOM, _data_path + "items/GoldenMushroom.png", _item_w);
    imread_vector(_item_imgs, ITEM_LIGHTNING, _data_path + "items/Lightning.png", _item_w);
    imread_vector(_item_imgs, ITEM_MIRROR, _data_path + "items/Mirror.png", _item_w);
    imread_vector(_item_imgs, ITEM_MUSHROOM, _data_path + "items/Mushroom.png", _item_w);
    imread_vector(_item_imgs, ITEM_REDSHELL, _data_path + "items/RedShell.png", _item_w);
    imread_vector(_item_imgs, ITEM_REDSHELL2, _data_path + "items/RedShell2.png", _item_w);
    imread_vector(_item_imgs, ITEM_REDSHELL3, _data_path + "items/RedShell3.png", _item_w);
    imread_vector(_item_imgs, ITEM_STAR, _data_path + "items/Star.png", _item_w);
    // load curses
    _curse_imgs.resize(NCURSES, cv::Mat3b(_item_w, _item_w, cv::Vec3b(0, 0, 255)));
    imread_vector(_curse_imgs, CURSE_BOO, _data_path + "items/BooCurse.png", _item_w);
    imread_vector(_curse_imgs, CURSE_DUD_START, _data_path + "items/DudStartCurse.png", _item_w);
    imread_vector(_curse_imgs, CURSE_GOLDENMUSHROOM, _data_path + "items/GoldenMushroomCurse.png", _item_w);
    imread_vector(_curse_imgs, CURSE_LIGHTNING, _data_path + "items/LightningCurse.png", _item_w);
    imread_vector(_curse_imgs, CURSE_MIRROR, _data_path + "items/MirrorCurse.png", _item_w);
    imread_vector(_curse_imgs, CURSE_MUSHROOM, _data_path + "items/MushroomCurse.png", _item_w);
    imread_vector(_curse_imgs, CURSE_REDSHELL_HIT, _data_path + "items/RedShellCurse.png", _item_w);
    imread_vector(_curse_imgs, CURSE_REDSHELL_COMING, _data_path + "items/RedShellComing.png", _item_w);
    imread_vector(_curse_imgs, CURSE_ROCKET_START, _data_path + "items/RocketStartCurse.png", _item_w);
    imread_vector(_curse_imgs, CURSE_STAR, _data_path + "items/StarCurse.png", _item_w);
    imread_vector(_curse_imgs, CURSE_TIMEBOMB_COUNTDOWN, _data_path + "items/TimeBombCountdown.png", _item_w);
    imread_vector(_curse_imgs, CURSE_TIMEBOMB_HIT, _data_path + "items/TimeBombCurse.png", _item_w);
    // load joypad statues
    _joypad_status_imgs.resize(NJOYPAD_STATUSES, cv::Mat3b(_item_w,_item_w, cv::Vec3b(0, 0, 255)));
    imread_vector(_joypad_status_imgs, JOYPAD_OK, _data_path + "warnings/joypadOK.png", _item_w);
    imread_vector(_joypad_status_imgs, JOYPAD_BAD_AXES_NB, _data_path + "warnings/joypadError.png", _item_w);
    imread_vector(_joypad_status_imgs, JOYPAD_BAD_BUTTONS_NB, _data_path + "warnings/joypadError.png", _item_w);
    imread_vector(_joypad_status_imgs, JOYPAD_NEVER_RECEIVED, _data_path + "warnings/joypadWarning.png", _item_w);
    imread_vector(_joypad_status_imgs, JOYPAD_TIMEOUT, _data_path + "warnings/joypadWarning.png", _item_w);
    // load robot statuses
    _robot_status_imgs.resize(NROBOT_STATUSES, cv::Mat3b(_item_w,_item_w, cv::Vec3b(0, 0, 255)));
    imread_vector(_robot_status_imgs, ROBOT_OK, _data_path + "warnings/robotOK.png", _item_w);
    imread_vector(_robot_status_imgs, ROBOT_NEVER_RECEIVED, _data_path + "warnings/robotWarning.png", _item_w);
    imread_vector(_robot_status_imgs, ROBOT_TIMEOUT, _data_path + "warnings/robotWarning.png", _item_w);
    // load lakitu statuses
    int lw = std::min(_gui_w, _gui_h);
    _lakitu_roi.width = _lakitu_roi.height = lw;
    _lakitu_roi.x = (_gui_w - lw) / 2;
    _lakitu_roi.y = (_gui_h - lw) / 2;
    _lakitu_status_imgs.resize(NLAKITU_STATUSES, cv::Mat3b(lw,lw, cv::Vec3b(0, 0, 255)));
    imread_vector(_lakitu_status_imgs, LAKITU_LIGHT0, _data_path + "lakitu/0.png", lw);
    imread_vector(_lakitu_status_imgs, LAKITU_LIGHT1, _data_path + "lakitu/1.png", lw);
    imread_vector(_lakitu_status_imgs, LAKITU_LIGHT2, _data_path + "lakitu/2.png", lw);
    imread_vector(_lakitu_status_imgs, LAKITU_LIGHT3, _data_path + "lakitu/3.png", lw);
    imread_vector(_lakitu_status_imgs, LAKITU_RACE_OVER, _data_path + "lakitu/finish.png", lw);

    restart_race();
  }

  //////////////////////////////////////////////////////////////////////////////

  bool restart_race() {
    _game_status = GAME_STATUS_WAITING;
    _lakitu_status = LAKITU_INVISIBLE;
    _last_lap_played = false;
    reset_players_curses_items();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool finish_race() {
    DEBUG_PRINT("Status: GAME_STATUS_RACE_OVER;");
    bool ok = (system("killall play") == 0);
    ok = ok && play_sound("you-win.mp3");
    _countdown.reset();
    _game_status = GAME_STATUS_RACE_OVER;
    _lakitu_status = LAKITU_RACE_OVER;
    reset_players_curses_items();
    return ok;
  } // end finish_race()

  //////////////////////////////////////////////////////////////////////////////

  bool refresh() {
    switch (_game_status) {
      case GAME_STATUS_WAITING:
        return refresh_waiting();
      case GAME_STATUS_COUNTDOWN:
        return refresh_countdown();
      case GAME_STATUS_RACE:
        return refresh_race();
      case GAME_STATUS_RACE_OVER:
      default:
        return refresh_race_over();
    }
  } // end refresh()

protected:
  //////////////////////////////////////////////////////////////////////////////

  //! check if each player has a publisher for joypad and a subscrier for cmd_vel
  //! \return true if everything OK, false otherwise.
  //! If false, the image needs to be displayed
  bool check_draw_joypads_robots() {
    bool checkok = true;
    for (unsigned int player_idx = 0; player_idx < _nplayers; ++player_idx) {
      Player* p = &(_players[player_idx]);
      // sanity checks: check joypad status
      if (p->joypad_status != JOYPAD_OK)
        checkok = false;
      else if (p->last_joy_updated.getTimeSeconds() > .5) {
        p->joypad_status = JOYPAD_TIMEOUT;
        checkok = false;
      }
      // sanity checks: check robot status
      if (p->cmd_vel_pub.getNumSubscribers())
        p->robot_status = ROBOT_OK;
      else {
        p->robot_status = ROBOT_TIMEOUT;
        checkok = false;
      }
      // now draw stuff
      unsigned int tlx = p->item_roi.x, tly = p->item_roi.y;
      // draw joypad status
      if (p->joypad_status != JOYPAD_OK)
        paste_img(_joypad_status_imgs[(int) p->joypad_status], _gui_final, tlx, tly);
      // draw robot status
      else if (p->robot_status != ROBOT_OK)
        paste_img(_robot_status_imgs[p->robot_status ], _gui_final, tlx, tly);
    } // end for player_idx
    return checkok;
  } // end check_draw_joypads_robots()

  //////////////////////////////////////////////////////////////////////////////

  void imshow() {
    DEBUG_PRINT("imshow()-%g s!", _race_timer.getTimeSeconds());
    cv::imshow("rosmariokart", _gui_final);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool refresh_waiting() {
    _gui_bg.copyTo(_gui_final);
    bool checkok = check_draw_joypads_robots();
    // check state changes
    if (checkok) { // start countdown
      DEBUG_PRINT("Status: GAME_STATUS_COUNTDOWN");
      play_sound("begin-race.mp3");
      _game_status = GAME_STATUS_COUNTDOWN;
      _lakitu_status = LAKITU_LIGHT0;
      _countdown.reset();
    }
    imshow();
    char c = cv::waitKey(50);
    if (c == 27)
      ros::shutdown();
    return checkok;
  } // end refresh_waiting()

  //////////////////////////////////////////////////////////////////////////////

  bool refresh_countdown() {
    bool checkok = check_draw_joypads_robots(), need_imshow = false;
    double time = _countdown.getTimeSeconds();
    int y = _lakitu_roi.y;
    // check state changes
    if (time >= 3 + 2.40 && _lakitu_status == LAKITU_LIGHT2) { // start race
      _race_timer.reset();
      _lakitu_status = LAKITU_LIGHT3;
      _game_status = GAME_STATUS_RACE;
      play_sound("battle-mode.mp3");
      // play a mushroom sound if needed
      bool play_dud = false, play_rocket = false;
      for (unsigned int i = 0; i < _nplayers; ++i) {
        play_dud = play_dud || (_players[i].curse == CURSE_DUD_START);
        play_rocket = play_rocket || (_players[i].curse == CURSE_ROCKET_START);
      }
      if (play_rocket) play_sound("boost.wav");
      if (play_dud) play_sound("spinout.wav");
      DEBUG_PRINT("Status: GAME_STATUS_RACE");
    }
    else if (time >= 3 + 1.20 && _lakitu_status == LAKITU_LIGHT1) {
      _lakitu_status = LAKITU_LIGHT2;
      need_imshow = true;
    }
    else if (time >= 3 && _lakitu_status == LAKITU_LIGHT0) {
      _lakitu_status = LAKITU_LIGHT1;
      play_sound("racestart.wav");
      need_imshow = true;
    }
    else if (_lakitu_status == LAKITU_LIGHT0) { // time in [0, 3 [
      double alpha = time / 3;
      y = (1 - alpha) * _gui_h + alpha * _lakitu_roi.y;
      need_imshow = true;
    }

    if (need_imshow) {
      paste_img(_lakitu_status_imgs[(int) _lakitu_status], _gui_final, _lakitu_roi.x, y);
      imshow();
    }
    char c = cv::waitKey(50);
    if (c == 27)
      ros::shutdown();
    return checkok;
  } // end refresh_countdown()

  //////////////////////////////////////////////////////////////////////////////

  bool refresh_race() {
    // check state changes
    if (_race_timer.getTimeSeconds() > _race_duration)
      finish_race();
    else if (_race_timer.getTimeSeconds() > _race_duration - 10
             && !_last_lap_played) {
      _last_lap_played = true;
      play_sound("last-lap.mp3");
    }

    // check joypads
    if (!check_draw_joypads_robots()) {
      imshow();
      char c = cv::waitKey(50);
      if (c == 27)
        ros::shutdown();
      return false;
    }

    // check items
    if (_last_roulette.getTimeSeconds() > _min_time_roulette && rand() % 100 >= 90)
      set_players_roulette();

    // check if each player item or curse is over
    for (unsigned int player_idx = 0; player_idx < _nplayers; ++player_idx) {
      Player* p = &(_players[player_idx]);
      // check items
      if (p->item == ITEM_ROULETTE) {
        if (p->roulette_timer.getTimeSeconds() > 3) // roulette timeout
          item_button_cb(player_idx, true);
        // rewind sound
        else if (_last_roulette_sound_play.getTimeSeconds() > .782) { // 0.782 seconds
          play_sound("itemreel.wav");
          _last_roulette_sound_play.reset();
        }
      } // end ITEM_ROULETTE

      // clean curses if needed
      double time_curse = p->curse_timer.getTimeSeconds();
      if (p->curse == CURSE_REDSHELL_COMING
          && time_curse > .1
          && (time_curse > _curse_timeout[CURSE_REDSHELL_COMING]
              || rand() % 50 == 0)) { // random end time
        play_sound("cpuspin.wav");
        p->play_animation("hit");
        p->play_animation_caster(player_idx, _players, "mock");
        p->receive_curse(CURSE_REDSHELL_HIT, p->curse_caster_idx);
      }
      else if (p->curse == CURSE_TIMEBOMB_COUNTDOWN
               && _timebomb.getTimeSeconds() > 4.935) { // 4.935 seconds
        p->play_animation("hit");
        p->play_animation_caster(player_idx, _players, "mock");
        p->receive_curse(CURSE_TIMEBOMB_HIT, p->curse_caster_idx);
      }
      else if ((p->curse != CURSE_NONE) // whatever curse
               && time_curse > _curse_timeout[p->curse]) {
        DEBUG_PRINT("Player %i: timeout on curse %i", player_idx, p->curse);
        p->receive_curse(CURSE_NONE, player_idx);
      }
    } // end for player_idx

    // draw what needs to be drawn
    bool need_imshow = false;
    if (_lakitu_status == LAKITU_LIGHT3
        || _lakitu_status == LAKITU_RACE_OVER) // will show lakitu going upwards -> need bg
      _gui_bg.copyTo(_gui_final);

    for (unsigned int i = 0; i < _nplayers; ++i) {
      Player* p = &(_players[i]);
      unsigned int tlx = p->item_roi.x, tly = p->item_roi.y;
      // redraw only if needed
      if (p->redraw_item_roi) {
        //ROS_WARN("Redraw player %i!", i);
        p->redraw_item_roi = false;
        need_imshow = true;
        if (p->curse != CURSE_NONE) {
          paste_img(_curse_imgs[(int) p->curse], _gui_final, tlx, tly);
        }
        else if (p->item == ITEM_ROULETTE) {
          paste_img(_item_imgs[random_item()], _gui_final, tlx, tly);
          p->redraw_item_roi = true; // redraw next frame
        }
        else if (p->item != ITEM_NONE) {
          paste_img(_item_imgs[p->item], _gui_final, tlx, tly);
        }
        else {// (CURSE_NONE && ITEM_NONE)
          cv::Rect roi(tlx, tly, _item_w, _item_w);
          cv::Mat3b item_roi(_gui_final(roi));
          _gui_bg(roi).copyTo(item_roi);
        }
        if (_lakitu_status != LAKITU_INVISIBLE) // if lakitu visible, need to redraw on each frame
          p->redraw_item_roi = true;
      } // end if redraw_item_roi
    } // end for i

    if (_lakitu_status == LAKITU_LIGHT3) { // show lakitu going upwards
      double alpha = _race_timer.getTimeSeconds() / 1.20;
      // lakitu_roi.y -> -lakitu_roi.height
      int y = (1. - alpha) * _lakitu_roi.y - alpha * _lakitu_roi.height;
      paste_img(_lakitu_status_imgs[(int) _lakitu_status], _gui_final, _lakitu_roi.x, y);
      need_imshow = true;
      if (_race_timer.getTimeSeconds() > 1.20)
        _lakitu_status = LAKITU_INVISIBLE;
    }

    if (need_imshow)
      imshow();
    char c = cv::waitKey(50);
    //DEBUG_PRINT("c:%i", c);
    // check commands
    if (c == 27)
      ros::shutdown();
    else if (c == 'a')
      item_button_cb(0, true);
    else if (c == 'z')
      item_button_cb(1, true);
    else if (c == 'q' || c == 's')
      set_players_roulette();
    else if (c == 81) // left
      sharp_turn_button_cb(-M_PI_2, 0, true);
    else if (c == 83) // right
      sharp_turn_button_cb(M_PI_2, 0, true);
    else if (c == 82) // up
      sharp_turn_button_cb(2 * M_PI, 0, true);
    else if (c == 84) // down
      sharp_turn_button_cb(-M_PI, 0, true);
    return true;
  } // end refresh_race()

  //////////////////////////////////////////////////////////////////////////////

  bool refresh_race_over() {
    bool checkok = check_draw_joypads_robots();
    double time = _countdown.getTimeSeconds();
    if (time <= 3) {
      double alpha = time / 3;
      int y = -(1-alpha) * _gui_h + alpha * _lakitu_roi.y;
      paste_img(_lakitu_status_imgs[(int) _lakitu_status], _gui_final, _lakitu_roi.x, y);
      imshow();
    }
    char c = cv::waitKey(50);
    if (c == 27)
      ros::shutdown();
    return checkok;
  } // end refresh_race_over()

  //////////////////////////////////////////////////////////////////////////////

  bool play_sound(const std::string & filename) const {
    std::ostringstream cmd;
    //cmd << "aplay --quiet " << _sound_path << filename << " &";
    cmd << "play --no-show-progress " << _sound_path << filename << " &";
    return (system(cmd.str().c_str()) == 0);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool set_players_roulette() {
    _last_roulette.reset();
    // set roulette to everybody
    for (unsigned int i = 0; i < _nplayers; ++i) {
      Player* p = &(_players[i]);
      // check player has no item or curse
      if (p->item != ITEM_NONE || p->curse != CURSE_NONE)
        continue;
      p->receive_item(ITEM_ROULETTE);
      p->roulette_timer.reset();
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  void reset_players_curses_items() {
    for (unsigned int i = 0; i < _nplayers; ++i) {
      _players[i].curse = CURSE_NONE;
      _players[i].item= ITEM_NONE;
      _players[i].redraw_item_roi = true;
    }
  }

  //////////////////////////////////////////////////////////////////////////////

  //! \player_idx starts at 0
  bool sharp_turn_button_cb(double angle_rad,
                            unsigned int player_idx,
                            bool skip_repetition_check = false) {
    if (player_idx >= _nplayers) // sanity check
      return false;
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
    if (player_idx >= _nplayers) // sanity check
      return false;
    // check what to do if Item_button
    Player* p = &(_players[player_idx]);
    if (!skip_repetition_check && p->item_button_before)
      return false;
    // cleverly find target_player_idx
    unsigned int target_player_idx = (player_idx + 1) % _nplayers; // safe value
    std::vector<unsigned int> possible_targets;
    for (unsigned int i = 0; i < _nplayers; ++i) {
      if (i != player_idx && _players[i].curse != CURSE_STAR)
        possible_targets.push_back(i);
    }
    if (!possible_targets.empty())
      target_player_idx = possible_targets[rand() % possible_targets.size()];
    Player*target = &(_players[target_player_idx]);
    Item pi = p->item;
    Curse pc = p->curse;
    Item targeti = target->item;
    Curse targetc = target->curse;

    // check curses
    if (pc == CURSE_TIMEBOMB_COUNTDOWN) { // passing timebomb
      if (targetc != CURSE_STAR) { // do nothing if target has star
        play_sound("menumove.wav");
        p->receive_curse(CURSE_NONE, player_idx);
        target->receive_curse(CURSE_TIMEBOMB_COUNTDOWN, player_idx);
      }
    } // end CURSE_TIMEBOMB_COUNTDOWN

    // check items
    if (pi == ITEM_BOO) { // swap items
      //play_sound("boo.wav");
      play_sound("boosteal.wav");
      p->receive_item(ITEM_NONE);
      if (is_real_item(targeti)) {
        p->receive_item(targeti);
        target->receive_item(ITEM_NONE);
        target->receive_curse(CURSE_BOO, player_idx);
      }
      else { // nothing to steal -> punish player!
        p->receive_curse(CURSE_BOO, player_idx);
      }
    }
    else if (pi == ITEM_GOLDENMUSHROOM) {
      play_sound("boost.wav");
      p->receive_item(ITEM_NONE);
      p->receive_curse(CURSE_GOLDENMUSHROOM, player_idx);
    }
    else if (pi == ITEM_LIGHTNING) {
      play_sound("lightning.wav");
      p->receive_item(ITEM_NONE);
      if (targetc != CURSE_STAR) { // do nothing if target has star
        target->receive_curse(CURSE_LIGHTNING, player_idx);
        p->play_animation("mock");
      }
    }
    else if (pi == ITEM_MIRROR) {
      play_sound("quartz.wav");
      p->receive_item(ITEM_NONE);
      if (targetc != CURSE_STAR) { // do nothing if target has star
        target->receive_curse(CURSE_MIRROR, player_idx);
        p->play_animation("mock");
      }
    }
    else if (pi == ITEM_MUSHROOM) {
      play_sound("boost.wav");
      p->receive_item(ITEM_NONE);
      p->receive_curse(CURSE_MUSHROOM, player_idx);
    }
    else if (pi == ITEM_REDSHELL || pi == ITEM_REDSHELL2 || pi == ITEM_REDSHELL3) {
      play_sound("cputhrow.wav");
      if (targetc != CURSE_STAR) // do nothing if target has star
        target->receive_curse(CURSE_REDSHELL_COMING, player_idx);
      if (pi == ITEM_REDSHELL) // decrease red shell counter
        p->receive_item(ITEM_NONE);
      else if (pi == ITEM_REDSHELL2)
        p->receive_item(ITEM_REDSHELL);
      else if (pi == ITEM_REDSHELL3)
        p->receive_item(ITEM_REDSHELL2);
    }
    else if (pi == ITEM_STAR) {
      play_sound("starman.wav");
      p->receive_item(ITEM_NONE);
      p->receive_curse(CURSE_STAR, player_idx);
    }
    else if (pi == ITEM_ROULETTE) { // got a new item
      if (pc == CURSE_NONE && drand48() < _timebomb_likelihood) { // new TIMEBOMB!
        play_sound("timebomb.wav");
        p->receive_item(ITEM_NONE);
        p->receive_curse(CURSE_TIMEBOMB_COUNTDOWN, player_idx);
        _timebomb.reset();
      } else {
        Item neww = random_item();
        p->receive_item(neww);
        play_sound("gotitem.wav");
      }
    } // end if ROULETTE

    p->item_button_before = true;
    return true;
  } // end item_button_cb()

  //////////////////////////////////////////////////////////////////////////////

  //! \player_idx starts at 0
  bool set_speed(unsigned int player_idx,
                 double v,
                 double w) {
    //DEBUG_PRINT("set_speed(%i, %g, %g)", player_idx, v, w);
    if (player_idx >= _nplayers) // sanity check
      return false;
    Player* p = &(_players[player_idx]);
    // now check for curses
    Curse c = p->curse;
    if (_game_status == GAME_STATUS_WAITING) {
      v = w = 0; // can't move during waiting
    } // end if GAME_STATUS_WAITING

    else if (_game_status == GAME_STATUS_COUNTDOWN) {
      if (c == CURSE_NONE && fabs(v) > .1) {
        if (fabs(_countdown.getTimeSeconds() - (3+1.2)) < .2) { // second red light -> rocket start
          DEBUG_PRINT("Player %i: rocket start!", player_idx);
          p->receive_curse(CURSE_ROCKET_START, player_idx);
        } else if (_countdown.getTimeSeconds() < 3+2.2) { // earlier than 200ms before green light -> dud start
          DEBUG_PRINT("Player %i: dud start!", player_idx);
          p->receive_curse(CURSE_DUD_START, player_idx);
        }
      } // end fabs(v) > .1
      v = w = 0; // can't move during countdown!
    } // end if GAME_STATUS_COUNTDOWN

    else if (_game_status == GAME_STATUS_RACE) {
      if (c == CURSE_DUD_START) {
        v *= .2; // sloowww
        w += .3 * p->scale_angular * cos(5*_race_timer.getTimeSeconds()); // oscillate
      }
      else if (c == CURSE_GOLDENMUSHROOM)
        v *= 2; // 200% faster
      else if (c == CURSE_LIGHTNING) {
        //ROS_WARN("%i:CURSE_LIGHTNING!", player_idx);
        v *= .2; // half speed
        w += .3 * p->scale_angular * cos(5*_race_timer.getTimeSeconds()); // oscillate
      }
      else if (c == CURSE_MIRROR) { // inverted commands
        v *= -1;
        w *= -1;
      }
      if (c == CURSE_MUSHROOM)
        v *= 2; // 200% faster
      else if (c == CURSE_REDSHELL_HIT || c == CURSE_TIMEBOMB_HIT)
        v = w = 0; // no move
      else if (c == CURSE_ROCKET_START)
        v *= 2; // 200% faster
      else if (c == CURSE_STAR)
        v *= 1.2; // 20% faster
    } // end if GAME_STATUS_RACE

    else if (_game_status == GAME_STATUS_RACE_OVER) {
      v = w = 0; // can't move when race over
    } // end if GAME_STATUS_RACE_OVER


    geometry_msgs::Twist vel;
    vel.linear.x = v;
    vel.angular.z = w;
    p->cmd_vel_pub.publish(vel);
    return true;
  } // end set_speed()

  //////////////////////////////////////////////////////////////////////////////

  //! \player_idx starts at 0
  void joy_cb(const sensor_msgs::Joy::ConstPtr& joy,
              unsigned int player_idx) {
    //DEBUG_PRINT("joy_cb(%i)", player_idx);
    if (player_idx >= _nplayers) // sanity check
      return;
    Player* p = &(_players[player_idx]);
    int naxes = joy->axes.size(), nbuttons = joy->buttons.size();
    if (naxes < _axis_90turn
        || naxes < _axis_180turn
        || naxes < _axis_linear
        || naxes < _axis_angular) {
      ROS_WARN_THROTTLE(1, "Only %i axes on joypad #%i!", naxes, player_idx);
      p->joypad_status = JOYPAD_BAD_AXES_NB;
      return;
    }
    if (nbuttons < _button_item) {
      ROS_WARN_THROTTLE(1, "Only %i buttons on joypad #%i!", nbuttons, player_idx);
      p->joypad_status = JOYPAD_BAD_BUTTONS_NB;
      return;
    }

    if (p->joypad_status != JOYPAD_OK) // redraw item on joystick reconnect
      p->redraw_item_roi = true;
    p->joypad_status = JOYPAD_OK;
    p->last_joy_updated.reset();
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
    else {
      p->item_button_before = false;
    }

    // cheat: trigger ROULETTE with button 0
    //if (joy->buttons[0]) set_players_roulette();

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
      item = ITEM_NONE;
      curse = CURSE_NONE;
      joypad_status = JOYPAD_NEVER_RECEIVED;
      robot_status = ROBOT_NEVER_RECEIVED;
      scale_linear = scale_angular = 1;
      item_button_before = sharp_turn_before = false;
      redraw_item_roi = true;
    }
    void receive_item(Item i) {
      item = i;
      redraw_item_roi = true;
    }
    void receive_curse(Curse c, unsigned int curse_caster_) {
      curse = c;
      curse_timer.reset();
      curse_caster_idx = curse_caster_;
      redraw_item_roi = true;
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
    bool redraw_item_roi;
    ros::Subscriber joy_sub;
    ros::Publisher cmd_vel_pub, sharp_turn_pub, animation_pub;
    Item item;
    Curse curse;
    JoypadStatus joypad_status;
    RobotStatus robot_status;
    bool sharp_turn_before, item_button_before;
    double scale_angular, scale_linear;
    Timer curse_timer, roulette_timer, last_joy_updated;
    unsigned int curse_caster_idx;
  }; // end struct Player //////////////////////////////////////////////////////

  unsigned int _nplayers;
  std::vector<Player> _players;
  GameStatus _game_status;
  LakituStatus _lakitu_status;
  double _min_time_roulette, _timebomb_likelihood;
  int _axis_linear, _axis_angular, _axis_90turn, _axis_180turn, _button_item;
  Timer _last_roulette_sound_play, _timebomb, _last_roulette, _countdown, _race_timer;
  Timer::Time _race_duration;
  bool _last_lap_played;

  // ros stuff
  ros::NodeHandle _nh_public, _nh_private;

  // opencv stuff
  int _gui_w, _gui_h;
  cv::Rect _lakitu_roi;
  cv::Mat3b _gui_bg, _gui_final;
  std::vector<cv::Mat3b> _item_imgs, _curse_imgs, _joypad_status_imgs,
  _robot_status_imgs, _lakitu_status_imgs;

  // items
  std::string _data_path, _sound_path;
  unsigned int _item_w; // pixels
  std::vector<double> _curse_timeout;
}; // end class Rosmariokart

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosmariokart");
  srand(time(NULL));
  srand48(time(NULL));
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
