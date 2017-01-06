/*!
  \file        rosmariokart.cpp
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \author      Eric MOLINE <molineer@gmail.com>
                -- PRISME University of Orleans
  \date        first release 2016/03/07
  \date        last release  2016/12/09
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

#include "rosmariokart/rosmariokart.h"
#include "rosmariokart/sdl_utils.h"

// third parties
#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <image_transport/image_transport.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

class Game {
public:
  class Player {
  public:
    Player(Game* game,
           const std::string & name,
           const unsigned int & player_idx)
      :
        _rgb_mutex(new boost::mutex()) { // ctor
      _game = game;
      _name = name;
      _player_idx = player_idx;
      // determine background color
      switch (player_idx % 4) {
        case 0:
          _bgcolor = cv::Vec4i(121, 28, 248, 255); break; // red
        case 1:
          _bgcolor = cv::Vec4i(22, 124, 78, 255); break; // green
        case 2:
          _bgcolor = cv::Vec4i(233, 109, 20, 255); break; // blue
        case 3:
        default:
          _bgcolor = cv::Vec4i(255, 255, 0, 255); break; // ?
          break;
      }
      reset();
    }

    ////////////////////////////////////////////////////////////////////////////

    void reset() {
      _item = ITEM_NONE;
      _curse = CURSE_NONE;
      _twist_status = TWIST_NEVER_RECEIVED;
      _item_button_status = ITEM_BUTTON_NEVER_RECEIVED;
      _robot_status = ROBOT_NEVER_RECEIVED;
      // rendering stuff
      force_next_render();
    }


    ////////////////////////////////////////////////////////////////////////////

    void compute_image_locations(int number_of_cols) {
      unsigned int row = _player_idx / number_of_cols,
          col = _player_idx % number_of_cols;
      _tl_win   = Point2d( col * _game->_player_w,
                           row * _game->_player_h );
      unsigned int padding = 10; // pixels
      _tl_item.x = _tl_win.x + _game->_player_w - _game->_item_w - padding;
      _tl_item.y = _tl_win.y + padding;
      _tl_curse.x = std::min(_tl_win.x + 0.5 * _game->_player_w - _game->_item_w/2,
                             _tl_item.x - _game->_item_w * 1.05) ;
      _tl_curse.y = _tl_item.y ;
    }

    ////////////////////////////////////////////////////////////////////////////

    bool load_avatar() {
      std::string imgname = "random_robot.png";
      if (_name.find("mip") != std::string::npos)
        imgname = "white_mip_black_bg.png";
      else if (_name.find("stage") != std::string::npos)
        imgname = "stage_black_bg.png";
      else if (_name.find("sumo") != std::string::npos)
        imgname = "white_sumo_black_bg.png";
      std::string fullfilename = _game->_data_path + std::string("robots/") + imgname;

      double avatar_w = 0.8 * std::min (_game->_player_w, _game->_player_h);
      if (!_avatar.from_file(_game->_renderer, fullfilename, avatar_w))
        return false;
      // we compute the top left corner to render the avatar centered
      _tl_avatar.x = _tl_win.x + (_game->_player_w - _avatar.get_width()) / 2;
      _tl_avatar.y = _tl_win.y + (_game->_player_h - _avatar.get_height()) / 2;
      return true;
    }

    ////////////////////////////////////////////////////////////////////////////

    void create_pub_sub() {
      ros::NodeHandle nh_public, nh_private("~");
      // create subscribers
      _twist_sub = nh_public.subscribe
          (_name + "/cmd_vel_raw", 1, &Player::twist_cb, this);
      _item_sub = nh_public.subscribe
          (_name + "/item", 1, &Player::item_cb, this);
      bool use_rgb = true;
      nh_private.param("use_rgb",use_rgb, use_rgb);
      if (use_rgb)
        _rgb_sub = _game->_it.subscribe
            (_name + "/image", 1, &Player::rgb_cb, this);
      // create publishers
      _twist_pub  = nh_public.advertise<geometry_msgs::Twist>
          (_name + "/cmd_vel", 1);
      _animation_pub  = nh_public.advertise<std_msgs::String>
          (_name + "/animation", 1);
      ROS_INFO("Robot '%s': getting twists on '%s', items on '%s', RGB on '%s' (used:%i),"
               "publishing twists on '%s', animations on '%s'",
               _name.c_str(), _twist_sub.getTopic().c_str(), _item_sub.getTopic().c_str(),
               _rgb_sub.getTopic().c_str(), use_rgb,
               _twist_pub.getTopic().c_str(), _animation_pub.getTopic().c_str());
    }

    ////////////////////////////////////////////////////////////////////////////

    const std::string & get_name() const { return _name; }
    Item get_item() const { return _item; }
    Curse get_curse() const { return _curse; }

    ////////////////////////////////////////////////////////////////////////////

    void receive_item(Item i) {
      if (i != _item)
        force_next_render();
      _item = i;
      if (i == ITEM_ROULETTE) {
        _game->play_sound("itemreel.wav");
        _game->_last_roulette_sound_play.reset();
        _roulette_timer.reset();
      }
    }

    ////////////////////////////////////////////////////////////////////////////

    void update_race() {
      // check items
      if (_item == ITEM_ROULETTE) {
        force_next_render();
        if (_roulette_timer.getTimeSeconds() > 3) // roulette timeout
          item_button_cb();
        // rewind sound
        else if (_game->_theme_name == "supermariokart"
                 && _game->_last_roulette_sound_play.getTimeSeconds() > .782) { // 0.782 seconds
          _game->play_sound("itemreel.wav");
          _game->_last_roulette_sound_play.reset();
        }
      } // end ITEM_ROULETTE

      // clean curses if needed
      double time_curse = _curse_timer.getTimeSeconds();
      if (_curse == CURSE_TIMEBOMB_COUNTDOWN
               && _game->_timebomb.getTimeSeconds() > 4.935) { // 4.935 seconds
        _game->_flying_curses.push_back(FlyingCurse(_game, CURSE_TIMEBOMB_HIT, _curse_caster_idx, _player_idx,
                                                    "mock", "hit"));
      }
      else if ((_curse != CURSE_NONE) // whatever curse
               && time_curse > _game->_curse_timeout[_curse]) {
        DEBUG_PRINT("Player %i: timeout on curse %i\n", _player_idx, _curse);
        receive_curse(CURSE_NONE, _player_idx);
      }
    }

    ////////////////////////////////////////////////////////////////////////////

    //! check if player has a publisher for twist and a subscriber for twist
    //! \return true if everything OK, false otherwise.
    //! If false, the image needs to be displayed
    bool check_twists_and_robots() {
      bool retval = true;

      // sanity checks: check item_button status
      if (_item_button_status != ITEM_BUTTON_OK)
        retval = false;

      // sanity checks: check twist status
      if (_twist_status != TWIST_OK)
        retval = false;
      else if (_last_twist_updated.getTimeSeconds() > .5) {
        ROS_WARN("Player %i: twist_TIMEOUT", _player_idx);
        // force next rendering if needed
        if (_twist_status != TWIST_TIMEOUT)
          force_next_render();
        _twist_status = TWIST_TIMEOUT;
        retval = false;
      }

      // sanity checks: check robot status
      RobotStatus prev_robot_status = _robot_status;
      if (_twist_pub.getNumSubscribers() == 0) {
        ROS_WARN("Player %i: ROBOT_TIMEOUT, no subscriber on '%s'",
                 _player_idx, _twist_pub.getTopic().c_str());
        _robot_status = ROBOT_TIMEOUT;
        retval =false;
      }
      else {
        _robot_status = ROBOT_OK;
      }
      // force next rendering if needed
      if (prev_robot_status != _robot_status)
        force_next_render();
      return retval;
    } // end check_twists_and_robots()

    ////////////////////////////////////////////////////////////////////////////

    void force_next_render() { _force_next_render = true; }

    ////////////////////////////////////////////////////////////////////////////

    inline Point2d get_tl_win()   const { return _tl_win; }
    inline Point2d get_tl_item()  const { return _tl_item; }
    inline Point2d get_tl_curse() const { return _tl_curse; }

    ////////////////////////////////////////////////////////////////////////////

    bool render(SDL_Renderer* renderer) {
      // we always need to render if lakitu is on screen, as it hides players
      if (_game->_lakitu_status != LAKITU_INVISIBLE)
        force_next_render();
      // do not render if not needed
      if (!_force_next_render)
        return true;
      _force_next_render = false;
      // draw colored background
      SDL_SetRenderDrawColor( renderer,
                              _bgcolor[0], _bgcolor[1], _bgcolor[2], _bgcolor[3]);
      SDL_Rect rect;
      rect.x = _tl_win.x;
      rect.y = _tl_win.y;
      rect.w = _game->_player_w;
      rect.h = _game->_player_h;
      SDL_RenderFillRect(renderer, &rect); // fill the background

      // draw image camera if available
      bool ok = true;
      if (has_rgb()) {
        _rgb_mutex->lock();
        ok = ok && _rgb.render(renderer, _tl_camview);
        _rgb_mutex->unlock();
      }
      else{ // we render the avatar
        ok = ok && _avatar.render(renderer, _tl_avatar);
      }

      // draw boundary lines between players
      SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );
      SDL_RenderDrawRect(renderer,&rect);

      // draw twist status
      Point2d pcenter = _tl_win + Point2d(_game->_player_w / 2, _game->_player_h / 2);
      if (_twist_status != TWIST_OK) {
        DEBUG_PRINT("Rendering twist status %i", _twist_status);
        ok = ok && _game->_twist_status_imgs[_twist_status].render_center(renderer, pcenter);
      }
      // draw robot status
      else if (_robot_status != ROBOT_OK) {
        DEBUG_PRINT("Rendering robot status %i", _robot_status);
        ok = ok && _game->_robot_status_imgs[_robot_status ].render_center(renderer, pcenter);
      }
      // draw item_button status
      else if (_item_button_status != ITEM_BUTTON_OK) {
        DEBUG_PRINT("Rendering press_item");
        ok = ok && _game->_press_item.render_center(renderer, pcenter);
      }

      if (_game->_game_status != GAME_STATUS_RACE)
        return ok;

      ok = ok && _game->_item_bg_img.render(renderer,
                                            Point2d(_tl_item.x-5,_tl_item.y-5)); // Item Background

      //ROS_WARN("Redraw player %i!", i); // do nothing if no twist or robot
      //if (_twist_status != twist_OK || _robot_status != ROBOT_OK)
      // return ok;

      if (_curse != CURSE_NONE)
        ok = ok && _game->_curse_imgs[(int)_curse].render(renderer,_tl_curse);
      if (_item == ITEM_ROULETTE)
        ok = ok && _game->_item_imgs[random_item()].render(renderer,_tl_item);
      else if (_item != ITEM_NONE)
        ok = ok && _game->_item_imgs[_item].render(renderer,_tl_item);

      ok = ok && _game->_countdown_texture_black.render_center
          (renderer, Point2d(_tl_win.x + _game->_player_w/5+2, _tl_win.y+ _game->_player_h/6+2),0.4)
          && _game->_countdown_texture_red.render_center
          (renderer, Point2d(_tl_win.x + _game->_player_w/5, _tl_win.y+ _game->_player_h/6),0.4);
      return ok;
    } // end render()

    ////////////////////////////////////////////////////////////////////////////

    void receive_curse(Curse c, unsigned int curse_caster_) {
      if (c != _curse)
        force_next_render();
      _curse = c;
      _curse_timer.reset();
      _curse_caster_idx = curse_caster_;
    }

    ////////////////////////////////////////////////////////////////////////////

    void play_animation(const std::string & s) {
      std_msgs::String anim;
      anim.data = s;
      _animation_pub.publish(anim);
    }

    ////////////////////////////////////////////////////////////////////////////

    bool set_speed(double v, double w) {
      //DEBUG_PRINT("set_speed(%i, %g, %g)\n", _player_idx, v, w);

      // now check for curses
      Curse c =_curse;
      if (_game->_game_status == GAME_STATUS_WAITING) {
        v = w = 0; // can't move during waiting
      } // end if GAME_STATUS_WAITING

      else if (_game->_game_status == GAME_STATUS_COUNTDOWN) {
        if (c == CURSE_NONE && fabs(v) > .1) {
          if (fabs(_game->_countdown.getTimeSeconds() - (3+1.2)) < .2) { // second red light -> rocket start
            DEBUG_PRINT("Player %i: rocket start!\n", _player_idx);
            receive_curse(CURSE_ROCKET_START, _player_idx);
          } else if (_game->_countdown.getTimeSeconds() < 3+2.2) { // earlier than 200ms before green light -> dud start
            DEBUG_PRINT("Player %i: dud start!\n", _player_idx);
            receive_curse(CURSE_DUD_START, _player_idx);
            _game->play_sound("anvil.wav");
          }
        } // end fabs(v) > .1
        v = w = 0; // can't move during countdown!
      } // end if GAME_STATUS_COUNTDOWN

      else if (_game->_game_status == GAME_STATUS_RACE) {
        if (c == CURSE_DUD_START) {
          v *= .2; // sloowww
          w += .3 * cos(5*_game->_race_timer.getTimeSeconds()); // oscillate
        }
        else if (c == CURSE_GOLDENMUSHROOM)
          v *= 2; // 200% faster
        else if (c == CURSE_LIGHTNING) {
          //ROS_WARN("%i:CURSE_LIGHTNING!", _player_idx);
          v *= .2; // half speed
          w += .3 * cos(5*_game->_race_timer.getTimeSeconds()); // oscillate
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

      else if (_game->_game_status == GAME_STATUS_RACE_OVER) {
        v = w = 0; // can't move when race over
      } // end if GAME_STATUS_RACE_OVER

      geometry_msgs::Twist vel;
      vel.linear.x = v;
      vel.angular.z = w;
      _twist_pub.publish(vel);
      return true;
    } // end set_speed()


  private:

    ////////////////////////////////////////////////////////////////////////////

    inline unsigned int nplayers() const { return _game->_nplayers; }

    ////////////////////////////////////////////////////////////////////////////

    bool has_rgb() const { return !_rgb.empty(); }

    ////////////////////////////////////////////////////////////////////////////

    bool item_button_cb() {
      DEBUG_PRINT("item_button_cb(%i)\n", _player_idx);
      if (_item_button_status == ITEM_BUTTON_NEVER_RECEIVED)
        force_next_render();
      _item_button_status = ITEM_BUTTON_OK;
      if (_game->_game_status != GAME_STATUS_RACE) // nothing to do
        return true;
      // check what to do if Item_button

      // cleverly find target_player_idx
      unsigned int target_player_idx = (_player_idx + 1) % nplayers(); // safe value
      std::vector<unsigned int> possible_targets;
      for (unsigned int i = 0; i < nplayers(); ++i) {
        if (i != _player_idx && _game->_players[i]._curse != CURSE_STAR)
          possible_targets.push_back(i);
      }
      if (!possible_targets.empty())
        target_player_idx = possible_targets[rand() % possible_targets.size()];
      Player*target = &(_game->_players[target_player_idx]);
      Item pi =_item;
      Curse pc =_curse;
      Item targeti = target->_item;
      Curse targetc = target->_curse;

      // check curses
      if (pc == CURSE_TIMEBOMB_COUNTDOWN) { // passing timebomb
        if (targetc != CURSE_STAR) { // do nothing if target has star
          _game->play_sound("timebomb_pass.wav");
          receive_curse(CURSE_NONE, _player_idx);
          target->receive_curse(CURSE_TIMEBOMB_COUNTDOWN, _player_idx);
        }
      } // end CURSE_TIMEBOMB_COUNTDOWN

      // check items
      if (pi == ITEM_BOO) { // swap items
        if (is_real_item(targeti)) {
          _game->_flying_curses.push_back(FlyingCurse(_game, CURSE_BOO, _player_idx, target_player_idx,
                                                      "mock", "", "boo.wav"));
          receive_item(targeti);
          target->receive_item(ITEM_NONE);
        }
        else { // nothing to steal -> punish player!
          _game->_flying_curses.push_back(FlyingCurse(_game, CURSE_BOO, _player_idx, _player_idx,
                                                      "", "", "boo.wav"));
          receive_item(ITEM_NONE);
        }
      }
      else if (pi == ITEM_GOLDENMUSHROOM) {
        _game->play_sound("mushroom.wav");
        receive_item(ITEM_NONE);
        receive_curse(CURSE_GOLDENMUSHROOM, _player_idx);
      }
      else if (pi == ITEM_LIGHTNING) {
        receive_item(ITEM_NONE);
        if (targetc != CURSE_STAR) // do nothing if target has star
          _game->_flying_curses.push_back(FlyingCurse(_game, CURSE_LIGHTNING, _player_idx, target_player_idx,
                                                      "mock", "", "lightning.wav"));
      }
      else if (pi == ITEM_MIRROR) {
        receive_item(ITEM_NONE);
        if (targetc != CURSE_STAR) // do nothing if target has star
          _game->_flying_curses.push_back(FlyingCurse(_game, CURSE_MIRROR, _player_idx, target_player_idx,
                                                      "mock", "", "quartz.wav"));
      }
      else if (pi == ITEM_MUSHROOM) {
        _game->play_sound("mushroom.wav");
        receive_item(ITEM_NONE);
        receive_curse(CURSE_MUSHROOM, _player_idx);
      }
      else if (pi == ITEM_REDSHELL || pi == ITEM_REDSHELL2 || pi == ITEM_REDSHELL3) {
        if (targetc != CURSE_STAR) // do nothing if target has star
          _game->_flying_curses.push_back(FlyingCurse(_game,
                                                      CURSE_REDSHELL_HIT, _player_idx, target_player_idx,
                                                      "mock", "hit", "redshell_throw.wav", "redshell_hit.wav",
                                                      .2 + drand48() * 2.));
        if (pi == ITEM_REDSHELL) // decrease red shell counter
          receive_item(ITEM_NONE);
        else if (pi == ITEM_REDSHELL2)
          receive_item(ITEM_REDSHELL);
        else if (pi == ITEM_REDSHELL3)
          receive_item(ITEM_REDSHELL2);
      }
      else if (pi == ITEM_STAR) {
        _game->play_sound("star.wav");
        receive_item(ITEM_NONE);
        receive_curse(CURSE_STAR, _player_idx);
      }
      else if (pi == ITEM_ROULETTE) { // got a new item
        if (pc == CURSE_NONE && drand48() < _game->_timebomb_likelihood) { // new TIMEBOMB!
          _game->play_sound("timebomb.wav");
          receive_item(ITEM_NONE);
          receive_curse(CURSE_TIMEBOMB_COUNTDOWN, _player_idx);
          _game->_timebomb.reset();
        } else {
          Item neww = random_item();
          receive_item(neww);
          _game->play_sound("gotitem.wav");
        }
      } // end if ROULETTE

      return true;
    } // end item_button_cb()

    ////////////////////////////////////////////////////////////////////////////

    void twist_cb(const geometry_msgs::Twist::ConstPtr& twist) {
      if (_twist_status != TWIST_OK)
        force_next_render();
      _twist_status = TWIST_OK;
      _last_twist_updated.reset();
      set_speed( twist->linear.x, twist->angular.z);
    } // end twist_cb();


    ////////////////////////////////////////////////////////////////////////////

    void item_cb(const std_msgs::Empty::ConstPtr&) {
      item_button_cb();
    } // end item_cb();

    ////////////////////////////////////////////////////////////////////////////

    void rgb_cb(const sensor_msgs::ImageConstPtr& rgb) {
      _rgb_mutex->lock();
      bool ok = _rgb.from_ros_image(_game->_renderer, *rgb,
                                    _game->_player_w, _game->_player_h);
      _rgb_mutex->unlock();
      if (!ok) {
        ROS_WARN("Texture::from_ros_image() failed!");
        return;
      }
      force_next_render();
      int paddingx = _game->_player_w - _rgb._width;
      _tl_camview.x = _tl_win.x + paddingx/2;
      //update of the _tl_camview in order to center the video image
      if (_game->_player_w ==_rgb._width){ // Image has been scaled to max width
        _tl_camview.x =_tl_win.x;
        int paddingy = _game->_player_h -_rgb._height;
        _tl_camview.y =_tl_win.y + paddingy/2;
      }
      else{ // Image has been scaled to max height
        _tl_camview.y =_tl_win.y;
        int paddingx = _game->_player_w -_rgb._width;
        _tl_camview.x =_tl_win.x + paddingx/2;
      }
    } // end rgb_cb()

    ////////////////////////////////////////////////////////////////////////////

    Game* _game;
    std::string _name;
    unsigned int _player_idx;
    Item _item;
    Curse _curse;
    TwistStatus _twist_status;
    RobotStatus _robot_status;
    ItemButtonStatus _item_button_status;
    Timer _curse_timer, _roulette_timer, _last_twist_updated;
    unsigned int _curse_caster_idx;

    // rendering stuff
    bool _force_next_render;
    Point2d _tl_item, _tl_curse, _tl_win, _tl_camview, _tl_avatar;
    Texture _avatar, _rgb;
    boost::shared_ptr<boost::mutex> _rgb_mutex;
    cv::Vec4i _bgcolor;
    // ROS data
    ros::Subscriber _twist_sub, _item_sub;
    image_transport::Subscriber _rgb_sub;
    ros::Publisher _twist_pub, _animation_pub;
  }; // end class Player //////////////////////////////////////////////////////

  //////////////////////////////////////////////////////////////////////////////

  class FlyingCurse {
  public:
    //! version between two players: the caster (item) and the receiver (curse)
    FlyingCurse(Game* game,
                Curse curse,
                int caster_idx, int receiver_idx,
                std::string caster_anim = "",
                std::string receiver_anim = "",
                std::string sound_on_cast = "",
                std::string sound_on_receive = "",
                double traveltime = 1)
      : _game(game), _curse(curse),
        _caster_idx(caster_idx), _receiver_idx(receiver_idx),
        _caster_anim(caster_anim), _receiver_anim(receiver_anim),
        _sound_on_receive(sound_on_receive),
        _traveltime(traveltime) {
      _pbegin = _game->_players[caster_idx].get_tl_item();
      _pend = _game->_players[receiver_idx].get_tl_curse();
      _life_timer.reset();
      // play sound if needed
      if (!sound_on_cast.empty())
        _game->play_sound(sound_on_cast);
    }

    bool render(SDL_Renderer* renderer) {
      if (is_over())
        return true; // nothing to do
      bool ok = true;
      double t = _life_timer.getTimeSeconds() / _traveltime;
      Point2d curr;
      curr.x = t * _pend.x + (1-t) * _pbegin.x;
      curr.y = t * _pend.y + (1-t) * _pbegin.y;
      Texture* tex = &(_game->_curse_imgs[_curse]);
      ok = ok && tex->render(renderer, curr);

      // force_hidden_players_render
      unsigned int nplayers = _game->_nplayers;
      int iw = _game->_item_w, pw = _game->_player_w, ph = _game->_player_h;
      for (unsigned int i = 0; i < nplayers; ++i) {
        Point2d tl = _game->_players[i].get_tl_win();
        // ROS_WARN("Item %i: comparing (%g, %g) and (%g, %g)", _curse, curr.x, curr.y, tl.x, tl.y);
        if (curr.x + iw < tl.x // too much at the left, no overlap
            || curr.x > tl.x + pw // at the right, no overlap
            || curr.y + iw < tl.y // on top, no overlap
            || curr.y > tl.y + ph) // at the bottom, no overlap
          continue;
        //ROS_WARN("Item %i: forcing next render of player %i", _curse, i);
        _game->_players[i].force_next_render();
      } // end for i
      return ok;
    } // end render()

    inline bool is_over() const {
      return _life_timer.getTimeSeconds() > _traveltime;
    }

    inline bool apply_to_receiver() const {
      _game->_players[_receiver_idx].receive_curse(_curse, _caster_idx);
      // play animation if needed
      if (!_caster_anim.empty() && _caster_idx < _game->_nplayers)
        _game->_players[_caster_idx].play_animation(_caster_anim);
      if (!_receiver_anim.empty() && _receiver_idx < _game->_nplayers)
        _game->_players[_receiver_idx].play_animation(_receiver_anim);
      // play sound if needed
      if (!_sound_on_receive.empty())
        return _game->play_sound(_sound_on_receive);
      return true;
    }

  protected:
    Game* _game;
    Curse _curse;
    Timer _life_timer;
    unsigned int _caster_idx, _receiver_idx;
    std::string _caster_anim, _receiver_anim, _sound_on_receive;
    double _traveltime;
    Point2d _pbegin, _pend;
  }; // end class FlyingCurse

  //////////////////////////////////////////////////////////////////////////////

  Game() : _nh_private("~"), _it(_nh_public) {
  }

  bool init() {
    // gui params
    _nh_private.param("winw", _winw, 800);
    _nh_private.param("winh", _winh, 600);
    _nh_private.param("min_time_roulette", _min_time_roulette, 10.);
    _nh_private.param("timebomb_likelihood", _timebomb_likelihood, 0.10); // 10%
    _nh_private.param("race_duration", _race_duration, 79.); // seconds = 1 min 19
    _nh_private.param("number_of_cols", _number_of_cols, 2);

    // item params
    _curse_timeout.resize(NCURSES, 10);
    _nh_private.param("curse_boo_timeout", _curse_timeout[CURSE_BOO], 3.);
    _nh_private.param("curse_dud_start_timeout", _curse_timeout[CURSE_DUD_START], 3.);
    _nh_private.param("curse_goldenmushroom_timeout", _curse_timeout[CURSE_GOLDENMUSHROOM], 3.);
    _nh_private.param("curse_lightning_timeout", _curse_timeout[CURSE_LIGHTNING], 5.);
    _nh_private.param("curse_mirror_timeout", _curse_timeout[CURSE_MIRROR], 5.);
    _nh_private.param("curse_mushroom_timeout", _curse_timeout[CURSE_MUSHROOM], 3.);
    _nh_private.param("curse_redshell_hit_timeout", _curse_timeout[CURSE_REDSHELL_HIT], 3.);
    _nh_private.param("curse_rocket_start_timeout", _curse_timeout[CURSE_ROCKET_START], 5.);
    _nh_private.param("curse_star_timeout", _curse_timeout[CURSE_STAR], 3.130);
    _nh_private.param("curse_timebomb_hit_timeout", _curse_timeout[CURSE_TIMEBOMB_HIT], 3.);

    // init SDL
    if ( SDL_Init( SDL_INIT_EVERYTHING ) == -1 ) {
      std::cout << " Failed to initialize SDL : " << SDL_GetError() << std::endl;
      return false;
    }
    //Initialize PNG loading
    int imgFlags = IMG_INIT_PNG;
    if( !( IMG_Init( imgFlags ) & imgFlags ) ) {
      ROS_ERROR( "SDL_image could not initialize! SDL_image Error: %s", IMG_GetError() );
      return false;
    }
    //Initialize SDL_mixer
    if( Mix_OpenAudio( 44100, MIX_DEFAULT_FORMAT, 2, 2048 ) < 0 ) {
      ROS_ERROR( "SDL_mixer could not initialize! SDL_mixer Error: %s", Mix_GetError() );
      return false;
    }
    //Initialize SDL_ttf
    if( TTF_Init() == -1 ) {
      printf( "SDL_ttf could not initialize! SDL_ttf Error: %s", TTF_GetError() );
      return false;
    }

    // create window
    SDL_Rect windowRect = { 10, 10, _winw, _winh};
    _window = SDL_CreateWindow( "cars", windowRect.x, windowRect.y, _winw, _winh, 0 );
    if ( _window == NULL ) {
      std::cout << "Failed to create window : " << SDL_GetError();
      return false;
    }
    // create renderer
    _renderer = SDL_CreateRenderer( _window, -1, 0 );
    if ( _renderer == NULL ) {
      std::cout << "Failed to create renderer : " << SDL_GetError();
      return false;
    }
    // Set size of renderer to the same as window
    SDL_RenderSetLogicalSize( _renderer, _winw, _winh );
    // Set color of renderer to light_blue
    SDL_SetRenderDrawColor( _renderer, 150, 150, 255, 255 );

    // create the list of players that have a name
    int i=0;
    while (true) {
      std::ostringstream param_name;
      param_name << "player" << i+1 << "_name";
      std::string name = "";
      _nh_private.param( param_name.str(), name, name);
      if (name.empty())
        break;
      Player p(this, name, i);
      _players.push_back(p);
      i++;
    }//end while add players
    _nplayers = _players.size();

    // configure GUI depending on the numbers of player and of the number of col
    int n_line = ceilf(1.*_nplayers/_number_of_cols);
    int n_col= std::min(_number_of_cols, (int) _nplayers);
    DEBUG_PRINT("%i players in a GUI of (%i x %i)\n", _nplayers, n_line,n_col);
    _player_w = _winw/n_col;
    _player_h = _winh/n_line;
    _item_w = std::min(_winw/(4*n_col), _winh/(4*n_line));

    // init the players
    _data_path = ros::package::getPath("rosmariokart") + std::string("/data/");
    for (unsigned int i = 0; i < _nplayers; ++i) {
      Player* p = &(_players[i]);
      p->create_pub_sub();
      p->compute_image_locations(_number_of_cols);
      p->load_avatar();
    }

    // load items - requires the renderer to be ready and _item_w to be computed
    _theme_name = "supertuxkart";
    _nh_private.param("theme_name", _theme_name, _theme_name);
    if (!load_items(_item_w)){
      ROS_WARN("from init(): Error loading item images");
    }
    // load music and sounds
    _sound_path =  _data_path + std::string("sounds/") + _theme_name + "/";

    //Open the time font
    _countdown_font = TTF_OpenFont( (_data_path + "fonts/LCD2U___.TTF").c_str(), _winh / 4);
    if( _countdown_font == NULL ) {
      printf( "Failed to load font! SDL_ttf Error: %s\n", TTF_GetError() );
      return false;
    }
    _last_renderer_countdown_time = -1;

    return restart_race();
  }


  //////////////////////////////////////////////////////////////////////////////

  //! load Items
  bool load_items(int item_size) {
    this->_item_imgs.resize(NITEMS);
    bool ok = true;

    // load Items Background
    std::string item_path = _data_path + "/items/" + _theme_name + "/";
    ok = ok && _item_bg_img.from_file(_renderer, item_path + "Item_BG.png", item_size + 10);

    ok = ok && _item_imgs[ITEM_BOO].from_file(_renderer, item_path + "Boo.png", item_size);
    ok = ok && _item_imgs[ITEM_GOLDENMUSHROOM].from_file(_renderer, item_path + "GoldenMushroom.png", item_size);
    ok = ok && _item_imgs[ITEM_LIGHTNING].from_file(_renderer, item_path + "Lightning.png", item_size);
    ok = ok && _item_imgs[ITEM_MIRROR].from_file(_renderer, item_path + "Mirror.png", item_size);
    ok = ok && _item_imgs[ITEM_MUSHROOM].from_file(_renderer, item_path + "Mushroom.png", item_size);
    ok = ok && _item_imgs[ITEM_REDSHELL].from_file(_renderer, item_path + "RedShell.png", item_size);
    ok = ok && _item_imgs[ITEM_REDSHELL2].from_file(_renderer, item_path + "RedShell2.png", item_size);
    ok = ok && _item_imgs[ITEM_REDSHELL3].from_file(_renderer, item_path + "RedShell3.png", item_size);
    ok = ok && _item_imgs[ITEM_STAR].from_file(_renderer, item_path + "Star.png", item_size);
    // load curses
    _curse_imgs.resize(NCURSES);
    ok = ok && _curse_imgs[CURSE_BOO].from_file(_renderer, item_path + "BooCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_DUD_START].from_file(_renderer, item_path + "DudStartCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_GOLDENMUSHROOM].from_file(_renderer, item_path + "GoldenMushroomCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_LIGHTNING].from_file(_renderer, item_path + "LightningCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_MIRROR].from_file(_renderer, item_path + "MirrorCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_MUSHROOM].from_file(_renderer, item_path + "MushroomCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_REDSHELL_HIT].from_file(_renderer, item_path + "RedShellCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_ROCKET_START].from_file(_renderer, item_path + "RocketStartCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_STAR].from_file(_renderer, item_path + "StarCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_TIMEBOMB_COUNTDOWN].from_file(_renderer, item_path + "TimeBombCountdown.png", item_size);
    ok = ok && _curse_imgs[CURSE_TIMEBOMB_HIT].from_file(_renderer, item_path + "TimeBombCurse.png", item_size);
    // load item_button status
    unsigned int status_size = std::min(_player_w, _player_h) - 5;
    _press_item.from_file(_renderer, _data_path + "warnings/press_item.png", status_size);
    // load twist statues
    _twist_status_imgs.resize(NTWIST_STATUSES);
    ok = ok && _twist_status_imgs[TWIST_OK].from_file(_renderer, _data_path + "warnings/joypadOK.png", status_size);
    ok = ok && _twist_status_imgs[TWIST_NEVER_RECEIVED].from_file(_renderer, _data_path + "warnings/joypadWarning.png", status_size);
    ok = ok && _twist_status_imgs[TWIST_TIMEOUT].from_file(_renderer, _data_path + "warnings/joypadWarning.png", status_size);
    // load robot statuses
    _robot_status_imgs.resize(NROBOT_STATUSES);
    ok = ok && _robot_status_imgs[ROBOT_OK].from_file(_renderer, _data_path + "warnings/robotOK.png", status_size);
    ok = ok && _robot_status_imgs[ROBOT_NEVER_RECEIVED].from_file(_renderer, _data_path + "warnings/robotWarning.png", status_size);
    ok = ok && _robot_status_imgs[ROBOT_TIMEOUT].from_file(_renderer, _data_path + "warnings/robotWarning.png", status_size);
    // load lakitu statuses
    std::string lakitu_path = _data_path + "lakitu/" + _theme_name + "/";
    int lakitu_width = 0.4*std::min(_winw, _winh);
    _lakitu_center.x = _winw / 2;
    _lakitu_center.y = _winh / 2;
    _lakitu_status_imgs.resize(NLAKITU_STATUSES);
    DEBUG_PRINT("lakiu_roi:%gx%g\n", _lakitu_center.x, _lakitu_center.y);
    ok = ok && _lakitu_status_imgs[LAKITU_INVISIBLE].from_file(_renderer, lakitu_path + "0.png", lakitu_width);
    ok = ok && _lakitu_status_imgs[LAKITU_LIGHT0].from_file(_renderer, lakitu_path + "0.png", lakitu_width);
    ok = ok && _lakitu_status_imgs[LAKITU_LIGHT1].from_file(_renderer, lakitu_path + "1.png", lakitu_width);
    ok = ok && _lakitu_status_imgs[LAKITU_LIGHT2].from_file(_renderer, lakitu_path + "2.png", lakitu_width);
    ok = ok && _lakitu_status_imgs[LAKITU_LIGHT3].from_file(_renderer, lakitu_path + "3.png", lakitu_width);
    ok = ok && _lakitu_status_imgs[LAKITU_RACE_OVER].from_file(_renderer, lakitu_path + "finish.png", lakitu_width);

    if (!ok)
      return false;

    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool clean() {
    // clean chunks
    for(std::map<std::string, Mix_Chunk*>::iterator it = _chunks.begin(); it != _chunks.end(); ++it) {
      if (it->second)
        Mix_FreeChunk(it->second);
    }
    _chunks.clear();
    for(std::map<std::string, Mix_Music*>::iterator it = _musics.begin(); it != _musics.end(); ++it) {
      if (it->second)
        Mix_FreeMusic(it->second);
    }
    _musics.clear();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool restart_race() {
    DEBUG_PRINT("restart_race()\n");
    play_music("music_prerace.mp3");
    _game_status = GAME_STATUS_WAITING;
    _lakitu_status = LAKITU_INVISIBLE;
    _lakitu.set_angle(0);
    _last_lap_played = false;
    _flying_curses.clear();
    for (unsigned int i = 0; i < _nplayers; ++i)
      _players[i].reset();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool finish_race() {
    DEBUG_PRINT("Status: GAME_STATUS_RACE_OVER\n");
    _game_status = GAME_STATUS_RACE_OVER;
    Mix_HaltMusic();
    bool ok = play_sound("race_end.wav");
    _countdown.reset();
    _lakitu_status = LAKITU_RACE_OVER;
    _lakitu.set_texture(&_lakitu_status_imgs[_lakitu_status]);
    return ok;
  } // end finish_race()

  //////////////////////////////////////////////////////////////////////////////

  bool update() {
    // update with events
    SDL_Event event;
    while ( SDL_PollEvent( &event ) ) {
      if ( event.type == SDL_QUIT )
        return false;
      else if ( event.type == SDL_KEYDOWN ) {
        SDL_Keycode key = event.key.keysym.sym;
        if (key == SDLK_q)
          return false;
        else if (key == SDLK_r)
          restart_race();
        //        else if (key == SDLK_a)
        //          item_button_cb(0, true);
        //        else if (key == SDLK_z)
        //          item_button_cb(1, true);
        //        else if (key == SDLK_q || key == SDLK_s)
        //          set_players_roulette();
        //        else if (key == SDLK_LEFT) // left
        //          sharp_turn_button_cb(-M_PI_2, 0, true);
        //        else if (key == SDLK_RIGHT) // right
        //          sharp_turn_button_cb(M_PI_2, 0, true);
        //        else if (key == SDLK_UP) // up
        //          sharp_turn_button_cb(2 * M_PI, 0, true);
        //        else if (key == SDLK_DOWN) // down
        //          sharp_turn_button_cb(-M_PI, 0, true);
      } // end SDL_KEYDOWN
    } // end while ( SDL_PollEvent( &event ) )

    switch (_game_status) {
      case GAME_STATUS_WAITING:
        return update_waiting();
      case GAME_STATUS_COUNTDOWN:
        return update_countdown();
      case GAME_STATUS_RACE:
        return update_race();
      case GAME_STATUS_RACE_OVER:
      default:
        return update_race_over();
    }
  } // end update()

  //////////////////////////////////////////////////////////////////////////////

  bool create_render_thread() {
    _render_thread = boost::thread(boost::bind(&Game::render_thread, this));
    return true;
  }

  void render_thread() {
    ros::Rate rate(15); // 15 fps
    while (ros::ok()) {
      render();
      rate.sleep();
    }
  } // end render_thread();

  bool render() {
    DEBUG_PRINT("render()-%g s!\n", _race_timer.getTimeSeconds());
    SDL_SetRenderDrawColor( _renderer, 150, 150, 255, 255 ); // light blue background
    bool ok = true;

    // render time
    if (_game_status == GAME_STATUS_RACE) {
      int time = _race_duration + 1 - _race_timer.getTimeSeconds();
      ok = ok && render_countdown2texture(time);
    }
    // render each player
    for (unsigned int i = 0; i < _nplayers; ++i)
      ok = ok && _players[i].render(_renderer);

    if (_game_status == GAME_STATUS_COUNTDOWN) {
      ok = ok && _lakitu.render(_renderer);
    } // end GAME_STATUS_COUNTDOWN

    else if (_game_status == GAME_STATUS_RACE) {
      // render flying curses
      for (unsigned int i = 0; i < _flying_curses.size(); ++i) {
        if (_flying_curses[i].is_over()) {
          ok = ok  && _flying_curses[i].apply_to_receiver();
          _flying_curses.erase(_flying_curses.begin() + i);
          --i;
          continue;
        }
        ok = ok && _flying_curses[i].render(_renderer);
      } // end for i

      if (_lakitu_status == LAKITU_LIGHT3) // show lakitu going upwards
        ok = ok && _lakitu.render(_renderer);
    } // end GAME_STATUS_RACE

    else if (_game_status == GAME_STATUS_RACE_OVER) {
      ok = ok && _lakitu.render(_renderer);
    } // end GAME_STATUS_RACE_OVER

    SDL_RenderPresent( _renderer);
    return true;
  }

protected:
  //! check if player has a publisher for twist and a subscriber for twist
  //! \return true if everything OK, false otherwise.
  //! If false, the image needs to be displayed
  bool check_twists_and_robots() {
    bool retval = true;
    for (unsigned int player_idx = 0; player_idx < _nplayers; ++player_idx) {
      if (!_players[player_idx].check_twists_and_robots())
        retval = false;
    }
    return retval;
  } // end check_twists_and_robots()

  //////////////////////////////////////////////////////////////////////////////

  bool update_waiting() {
    bool checkok = check_twists_and_robots();
    // check state changes
    if (checkok) { // start countdown
      DEBUG_PRINT("Status: GAME_STATUS_COUNTDOWN\n");
      Mix_HaltMusic();
      play_sound("race_begin.wav");
      _game_status = GAME_STATUS_COUNTDOWN;
      _countdown.reset();
      _lakitu_status = LAKITU_LIGHT0;
      _lakitu.set_texture(&_lakitu_status_imgs[_lakitu_status]);
      _lakitu.set_position(Point2d(_lakitu_center.x, -_lakitu.get_height()));
    }
    return true;
  } // end update_waiting()

  //////////////////////////////////////////////////////////////////////////////

  bool update_countdown() {
    check_twists_and_robots();
    double time = _countdown.getTimeSeconds();
    // check state changes
    if (time >= 3 + 2.40 && _lakitu_status == LAKITU_LIGHT2) { // start race
      _race_timer.reset();
      _lakitu_status = LAKITU_LIGHT3;
      play_sound("light_countdown3.wav");
      play_music("music_race.mp3");
      _lakitu.set_texture(&_lakitu_status_imgs[_lakitu_status]);
      _game_status = GAME_STATUS_RACE;
      // play a mushroom sound if needed
      bool play_dud = false, play_rocket = false;
      for (unsigned int i = 0; i < _nplayers; ++i) {
        if (_players[i].get_curse() == CURSE_DUD_START) // refresh dud start
          _players[i].receive_curse(CURSE_DUD_START, i);
        play_dud = play_dud || (_players[i].get_curse() == CURSE_DUD_START);
        play_rocket = play_rocket || (_players[i].get_curse() == CURSE_ROCKET_START);
      }
      if (play_rocket) play_sound("mushroom.wav");
      if (play_dud) play_sound("dud_start.wav");
      DEBUG_PRINT("Status: GAME_STATUS_RACE\n");
    }
    else if (time >= 3 + 1.20 && _lakitu_status == LAKITU_LIGHT1) {
      _lakitu_status = LAKITU_LIGHT2;
      play_sound("light_countdown2.wav");
      _lakitu.set_texture(&_lakitu_status_imgs[_lakitu_status]);
    }
    else if (time >= 3 && _lakitu_status == LAKITU_LIGHT0) {
      _lakitu_status = LAKITU_LIGHT1;
      play_sound("light_countdown1.wav");
      _lakitu.set_texture(&_lakitu_status_imgs[_lakitu_status]);
      _lakitu.set_position(_lakitu_center); // just in case
    }
    // going downward: --_lakitu.get_height()) -> lakitu_center.y
    else if (_lakitu_status == LAKITU_LIGHT0){
      double alpha = time / 3.0;
      int y = (1. - alpha) * (-_lakitu.get_height()) + alpha * _lakitu_center.y;
      _lakitu.set_position(Point2d(_lakitu_center.x, y));
    }
    return true;
  } // end update_countdown()

  //////////////////////////////////////////////////////////////////////////////

  bool update_race() {
    if (_lakitu_status == LAKITU_LIGHT3) { // going upwards: lakitu_center.y -> -lakitu_center.h
      double time = _race_timer.getTimeSeconds(), alpha = time / 1.20;
      int y = (1. - alpha) * _lakitu_center.y - alpha * _lakitu.get_height();
      _lakitu.set_position(Point2d(_lakitu_center.x, y));
      if (time > 1.20)
        _lakitu_status = LAKITU_INVISIBLE;
    }

    // check state changes
    if (_race_timer.getTimeSeconds() > _race_duration)
      finish_race();
    else if (_race_timer.getTimeSeconds() > _race_duration - 10
             && !_last_lap_played) {
      _last_lap_played = true;
      play_sound("race_last_lap.wav");
    }

    // check twists
    if (!check_twists_and_robots())
      return true;

    // check items
    if (_last_roulette.getTimeSeconds() > _min_time_roulette && rand() % 100 >= 90)
      set_players_roulette();

    // check if each player item or curse is over
    for (unsigned int player_idx = 0; player_idx < _nplayers; ++player_idx) {
      Player* p = &(_players[player_idx]);
      p->update_race();
    } // end for _player_idx
    return true;
  } // end update_race()

  //////////////////////////////////////////////////////////////////////////////

  bool update_race_over() {
    check_twists_and_robots();
    int x = _lakitu_center.x, y = _lakitu_center.y;
    double time = _countdown.getTimeSeconds();
    if (time <= 3) { // get lakitu down
      double alpha = time / 3;
      y = (1. - alpha) * (-_lakitu.get_height()) + alpha * _lakitu_center.y;
    }
    else { // oscillate
      x += _winw /8. * sin(time);
      y -= _winh /20. * sin(2*time);
      _lakitu.set_angle(.05 * cos(time*4+1));
    }

    _lakitu.set_position(Point2d(x, y));
    return true;
  } // end update_race_over()

  //////////////////////////////////////////////////////////////////////////////

  bool play_sound(const std::string & filename) {
    if (filename.empty())
      return true;
    std::map<std::string, Mix_Chunk*>::iterator it = _chunks.find(filename);
    Mix_Chunk* chunk = it->second;
    if (it == _chunks.end()) {
      if (!(chunk = Mix_LoadWAV((_sound_path + filename).c_str()) )) {
        ROS_WARN( "Failed to load music '%s'! SDL_mixer Error: %s",
                  filename.c_str(), Mix_GetError() );
        return false;
      }
      _chunks.insert(std::make_pair(filename, chunk));
    }
    Mix_PlayChannel( -1, chunk, 0 );
    return true;
  } // end play_sound()

  //////////////////////////////////////////////////////////////////////////////

  bool play_music(const std::string & filename) {
    DEBUG_PRINT("play_music('%s')", filename.c_str());
    if (filename.empty())
      return true;
    std::map<std::string, Mix_Music*>::iterator it = _musics.find(filename);
    Mix_Music* music = it->second;
    if (it == _musics.end()) {
      if (!(music = Mix_LoadMUS((_sound_path + filename).c_str()) )) {
        ROS_WARN( "Failed to load music '%s'! SDL_mixer Error: %s",
                  filename.c_str(), Mix_GetError() );
        return false;
      }
      _musics.insert(std::make_pair(filename, music));
    }
    Mix_HaltMusic();
    Mix_VolumeMusic(128);
    Mix_PlayMusic(music, -1);
    return true;
  } // end play_music()

  //////////////////////////////////////////////////////////////////////////////

  //!\return true i  render OK or already done // This functionnality has been
  //! updated in order to have double color timer (for visibility)
  bool render_countdown2texture(const int time) {
    if (_last_renderer_countdown_time == time)
      return true;
    std::ostringstream time_str; time_str << time;
    _last_renderer_countdown_time = time;
    if (!_countdown_texture_red.from_rendered_text
        (_renderer, _countdown_font, time_str.str(), 255, 0, 0)
        || !_countdown_texture_black.from_rendered_text
        (_renderer, _countdown_font, time_str.str(), 9, 0, 0))
      return false;
    // force next rendering for all players
    for (unsigned int i = 0; i < _nplayers; ++i)
      _players[i].force_next_render();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool set_players_roulette() {
    _last_roulette.reset();
    // set roulette to everybody
    for (unsigned int i = 0; i < _nplayers; ++i) {
      Player* p = &(_players[i]);
      // check player has no item or curse
      if (p->get_item() != ITEM_NONE || p->get_curse() != CURSE_NONE)
        continue;
      p->receive_item(ITEM_ROULETTE);
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  unsigned int _nplayers;
  std::vector<Player> _players;
  GameStatus _game_status;
  double _min_time_roulette, _timebomb_likelihood;
  Timer _last_roulette_sound_play, _timebomb, _last_roulette, _countdown, _race_timer;
  Timer::Time _race_duration;
  bool _last_lap_played;
  LakituStatus _lakitu_status;

  // ros stuff
  ros::NodeHandle _nh_public, _nh_private;
  // allows subscription to image sent by camera using image_transport
  image_transport::ImageTransport _it;

  // sound stuff
  std::map<std::string, Mix_Chunk*> _chunks;
  std::map<std::string, Mix_Music*> _musics;

  // rendering stuff
  boost::thread _render_thread;
  Timer _last_render;
  SDL_Window* _window;
  SDL_Renderer* _renderer;
  int _winw, _winh, _number_of_cols;
  int _item_w;  // size of item icon, pixels
  int _player_w, _player_h; // size of player windows, pixels
  Point2d _lakitu_center ;
  Entity _lakitu;

  // time display stuff
  TTF_Font *_countdown_font;
  int _last_renderer_countdown_time;
  Texture _countdown_texture_red, _countdown_texture_black, _press_item;

  // shared textures
  std::string _data_path, _sound_path, _theme_name;
  std::vector<Texture> _lakitu_status_imgs, _item_imgs, _curse_imgs, _bg_imgs,
  _twist_status_imgs, _robot_status_imgs;

  // items
  Texture _item_bg_img;
  std::vector<double> _curse_timeout;
  std::vector<FlyingCurse> _flying_curses;
}; // end class Game

////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv) {
  ros::init(argc, argv, "rosmariokart");
  srand(time(NULL));
  srand48(time(NULL));
  Game game;
  if (!game.init()) {
    ROS_ERROR("game.init() failed!");
    return false;
  }
  //  if (!game.create_render_thread()) {
  //    ROS_ERROR("game.create_render_thread() failed!");
  //    return false;
  //  }
  ros::Rate update_rate(25);
  while (ros::ok()) {
    if (!game.update()) {
      ROS_ERROR("game.update() failed!");
      return false;
    }
    if (!game.render()){
      ROS_WARN("render failed!");
    }
    ros::spinOnce();
    if (!update_rate.sleep()){
      // ROS_WARN(" Main Cycle Rate non respected");
    }
  } // end while (ros::ok())
  return (game.clean() ? 0 : -1);
}
