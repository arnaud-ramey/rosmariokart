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

#include "rosmariokart/rosmariokart.h"
#include "rosmariokart/sdl_utils.h"
// third parties
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/package.h>

class Game {
public:

  Game() : _nh_private("~"), _it(_nh_public) {
  }

  bool init() {
    // gui params
    _nh_private.param("winw", _winw, 800);
    _nh_private.param("winh", _winh, 600);
    _nh_private.param("min_time_roulette", _min_time_roulette, 10.);
    _nh_private.param("timebomb_likelihood", _timebomb_likelihood, 0.03); // 3%
    _nh_private.param("race_duration", _race_duration, 79.); // seconds = 1 min 19
    // item params
    _curse_timeout.resize(NCURSES, 10);
    _nh_private.param("curse_boo_timeout", _curse_timeout[CURSE_BOO], 3.);
    _nh_private.param("curse_dud_start_timeout", _curse_timeout[CURSE_DUD_START], 3.);
    _nh_private.param("curse_goldenmushroom_timeout", _curse_timeout[CURSE_GOLDENMUSHROOM], 3.);
    _nh_private.param("curse_lightning_timeout", _curse_timeout[CURSE_LIGHTNING], 5.);
    _nh_private.param("curse_mirror_timeout", _curse_timeout[CURSE_MIRROR], 5.);
    _nh_private.param("curse_mushroom_timeout", _curse_timeout[CURSE_MUSHROOM], 3.);
    _nh_private.param("curse_redshell_coming_timeout", _curse_timeout[CURSE_REDSHELL_COMING], 3.);
    _nh_private.param("curse_redshell_hit_timeout", _curse_timeout[CURSE_REDSHELL_HIT], 3.);
    _nh_private.param("curse_rocket_start_timeout", _curse_timeout[CURSE_ROCKET_START], 5.);
    _nh_private.param("curse_star_timeout", _curse_timeout[CURSE_STAR], 3.130);
    _nh_private.param("curse_timebomb_hit_timeout", _curse_timeout[CURSE_TIMEBOMB_HIT], 3.);

    // player params
    Player p1(this, 0), p2(this, 1), p3(this, 2), p4(this, 3);
    _nh_private.param("player1_name", p1._name, std::string("player1"));
    _players.push_back(p1);
    _nh_private.param("player2_name", p2._name, std::string(""));
    if (p2._name.length() > 0)
      _players.push_back(p2);
    _nh_private.param("player3_name", p3._name, std::string(""));
    if (p3._name.length() > 0)
      _players.push_back(p3);
    _nh_private.param("player4_name", p4._name, std::string(""));
    if (p4._name.length() > 0)
      _players.push_back(p4);
    _nplayers = _players.size();

    for (unsigned int i = 0; i < _nplayers; ++i) {
      Player* p = &(_players[i]);
      // create subscribers - pass i
      // http://ros-users.122217.n3.nabble.com/How-to-identify-the-subscriber-or-the-topic-name-related-to-a-callback-td2391327.html
      p->create_pub_sub();
    }

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
    window = SDL_CreateWindow( "cars", windowRect.x, windowRect.y, _winw, _winh, 0 );
    if ( window == NULL ) {
      std::cout << "Failed to create window : " << SDL_GetError();
      return false;
    }
    // create renderer
    renderer = SDL_CreateRenderer( window, -1, 0 );
    if ( renderer == NULL ) {
      std::cout << "Failed to create renderer : " << SDL_GetError();
      return false;
    }
    // Set size of renderer to the same as window
    SDL_RenderSetLogicalSize( renderer, _winw, _winh );
    // Set color of renderer to light_blue
    SDL_SetRenderDrawColor( renderer, 150, 150, 255, 255 );

    // alloc data
    _data_path = ros::package::getPath("rosmariokart") + std::string("/data/");
    _sound_path =  _data_path + std::string("sounds/");
    // load music and sounds
    // WAVE, MOD, MIDI, OGG, MP3, FLAC
    // sox cocoa_river.ogg -r 22050 cocoa_river.wav
    if( !(_music = Mix_LoadMUS( (_sound_path + "battle-mode.mp3").c_str() )) ) {
      ROS_ERROR( "Failed to load music! SDL_mixer Error: %s", Mix_GetError() );
      return false;
    }
    Mix_VolumeMusic(128);
    //Open the time font
    _countdown_font = TTF_OpenFont( (_data_path + "fonts/LCD2U___.TTF").c_str(), _winh / 4);
    if( _countdown_font == NULL ) {
      printf( "Failed to load font! SDL_ttf Error: %s\n", TTF_GetError() );
      return false;
    }
    _last_renderer_countdown_time = -1;

    // configure GUI
    _item_w = std::min(_winw/4, _winh/4);


    if (_nplayers == 1) {
      player_w =_winw;
      player_h = _winh;
      _players[0]._item_tl_corner = Point2d(_winw - _item_w*1.1, 10);
      _players[0]._tl_win = Point2d(0, 0);

    }
    else if (_nplayers == 2) {
      // Test if image have to be splitted in Right-Left or Up-Down
      int _screen_main_direction = std::max(_winw,_winh);

      if (_screen_main_direction == _winw){ //Split main screnn in Right_Left

        player_w =_winw/2;
        player_h = _winh;

        _players[0]._item_tl_corner = Point2d(_winw/2 - _item_w*1.1 , 10);
        _players[0]._tl_win = Point2d(0, 0);

        _players[1]._item_tl_corner = Point2d(_winw - _item_w*1.1 , 10);
        _players[1]._tl_win = Point2d(_winw/2, 0);

      }
      else{ //Split main screnn in Up_Down
        player_w =_winw;
        player_h = _winh/2;

        _players[0]._item_tl_corner = Point2d(_winw - _item_w*1.1, 10);
        _players[0]._tl_win = Point2d(0, 0);

        _players[1]._item_tl_corner = Point2d(_winw - _item_w*1.1,_winh/2 + 10 );
        _players[1]._tl_win = Point2d(0,_winh/2);
      }

    }
    else if (_nplayers == 3 || _nplayers == 4) {
      _item_w = std::min(_winw / 6, _winh / 6);

      player_w =_winw/2;
      player_h = _winh/2;

      _players[0]._item_tl_corner = Point2d(_winw/2 - _item_w*1.1, 10);
      _players[0]._tl_win = Point2d(0, 0);

      _players[1]._item_tl_corner = Point2d(_winw - _item_w*1.1, 10 );
      _players[1]._tl_win = Point2d(_winw/2,0);

      _players[2]._item_tl_corner = Point2d(_winw/2 - _item_w*1.1, _winh/2 + 10);
      _players[2]._tl_win = Point2d(0,_winh/2);

      if (_nplayers == 4)
        _players[3]._item_tl_corner = Point2d(_winw - _item_w*1.1,_winh/2 + 10);
      _players[3]._tl_win = Point2d(_winw/2,_winh/2);

    }
    // put robot background

    for (unsigned int i = 0; i < _nplayers; ++i) {
      Player* p = &(_players[i]);
      std::string imgname = "random_robot.png";
      if (p->_name.find("mip") != std::string::npos)
        imgname = "white_mip_black_bg.png";
      else if (p->_name.find("robot_") != std::string::npos)
        imgname = "stage_black_bg.png";
      else if (p->_name.find("sumo") != std::string::npos)
        imgname = "white_sumo_black_bg.png";
      std::string fullfilename = _data_path + std::string("robots/") + imgname;

      double avatar_w = 0.8 * std::min (player_w,player_h);

      p->_avatar.from_file(renderer, fullfilename, avatar_w);

      p->_curse_tl.x = std::min(p->_tl_win.x + 0.5*player_w - _item_w/2,
                                p->_item_tl_corner.x - _item_w*1.05) ;
      p->_curse_tl.y = p->_item_tl_corner.y ;

    } // end for i

    if (! load_item(_item_w)){
      ROS_WARN("from init(): Error loading item images");
    }
    return restart_race();
  }


  //////////////////////////////////////////////////////////////////////////////

  bool load_item(int item_size) {

    // load Items
    this->_item_imgs.resize(NITEMS);
    this->_bg_imgs.resize(NBG);
    bool ok = true;

    // load Items Background
    ok = ok && _bg_imgs[BG_ITEMS].from_file(renderer, _data_path + "items/Item_BG.png", item_size + 10);

    ok = ok && _item_imgs[ITEM_BOO].from_file(renderer, _data_path + "items/Boo.png", item_size);
    ok = ok && _item_imgs[ITEM_GOLDENMUSHROOM].from_file(renderer, _data_path + "items/GoldenMushroom.png", item_size);
    ok = ok && _item_imgs[ITEM_LIGHTNING].from_file(renderer, _data_path + "items/Lightning.png", item_size);
    ok = ok && _item_imgs[ITEM_MIRROR].from_file(renderer, _data_path + "items/Mirror.png", item_size);
    ok = ok && _item_imgs[ITEM_MUSHROOM].from_file(renderer, _data_path + "items/Mushroom.png", item_size);
    ok = ok && _item_imgs[ITEM_REDSHELL].from_file(renderer, _data_path + "items/RedShell.png", item_size);
    ok = ok && _item_imgs[ITEM_REDSHELL2].from_file(renderer, _data_path + "items/RedShell2.png", item_size);
    ok = ok && _item_imgs[ITEM_REDSHELL3].from_file(renderer, _data_path + "items/RedShell3.png", item_size);
    ok = ok && _item_imgs[ITEM_STAR].from_file(renderer, _data_path + "items/Star.png", item_size);
    // load curses
    _curse_imgs.resize(NCURSES);
    ok = ok && _curse_imgs[CURSE_BOO].from_file(renderer, _data_path + "items/BooCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_DUD_START].from_file(renderer, _data_path + "items/DudStartCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_GOLDENMUSHROOM].from_file(renderer, _data_path + "items/GoldenMushroomCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_LIGHTNING].from_file(renderer, _data_path + "items/LightningCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_MIRROR].from_file(renderer, _data_path + "items/MirrorCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_MUSHROOM].from_file(renderer, _data_path + "items/MushroomCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_REDSHELL_HIT].from_file(renderer, _data_path + "items/RedShellCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_REDSHELL_COMING].from_file(renderer, _data_path + "items/RedShellComing.png", item_size);
    ok = ok && _curse_imgs[CURSE_ROCKET_START].from_file(renderer, _data_path + "items/RocketStartCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_STAR].from_file(renderer, _data_path + "items/StarCurse.png", item_size);
    ok = ok && _curse_imgs[CURSE_TIMEBOMB_COUNTDOWN].from_file(renderer, _data_path + "items/TimeBombCountdown.png", item_size);
    ok = ok && _curse_imgs[CURSE_TIMEBOMB_HIT].from_file(renderer, _data_path + "items/TimeBombCurse.png", item_size);
    // load joypad statues
    _joypad_status_imgs.resize(NJOYPAD_STATUSES);
    ok = ok && _joypad_status_imgs[JOYPAD_OK].from_file(renderer, _data_path + "warnings/joypadOK.png", item_size);
    ok = ok && _joypad_status_imgs[JOYPAD_BAD_AXES_NB].from_file(renderer, _data_path + "warnings/joypadError.png", item_size);
    ok = ok && _joypad_status_imgs[JOYPAD_BAD_BUTTONS_NB].from_file(renderer, _data_path + "warnings/joypadError.png", item_size);
    ok = ok && _joypad_status_imgs[JOYPAD_NEVER_RECEIVED].from_file(renderer, _data_path + "warnings/joypadWarning.png", item_size);
    ok = ok && _joypad_status_imgs[JOYPAD_TIMEOUT].from_file(renderer, _data_path + "warnings/joypadWarning.png", item_size);
    // load robot statuses
    _robot_status_imgs.resize(NROBOT_STATUSES);
    ok = ok && _robot_status_imgs[ROBOT_OK].from_file(renderer, _data_path + "warnings/robotOK.png", item_size);
    ok = ok && _robot_status_imgs[ROBOT_NEVER_RECEIVED].from_file(renderer, _data_path + "warnings/robotWarning.png", item_size);
    ok = ok && _robot_status_imgs[ROBOT_TIMEOUT].from_file(renderer, _data_path + "warnings/robotWarning.png", item_size);
    // load lakitu statuses
    int lakitu_width = 0.4*std::min(_winw, _winh);
    _lakitu_center.x = _winw / 2;
    _lakitu_center.y = _winh / 2;
    _lakitu_status_imgs.resize(NLAKITU_STATUSES);
    DEBUG_PRINT("lakiu_roi:%gx%g\n", _lakitu_center.x, _lakitu_center.y);
    ok = ok && _lakitu_status_imgs[LAKITU_INVISIBLE].from_file(renderer, _data_path + "lakitu/0.png", lakitu_width);
    ok = ok && _lakitu_status_imgs[LAKITU_LIGHT0].from_file(renderer, _data_path + "lakitu/0.png", lakitu_width);
    ok = ok && _lakitu_status_imgs[LAKITU_LIGHT1].from_file(renderer, _data_path + "lakitu/1.png", lakitu_width);
    ok = ok && _lakitu_status_imgs[LAKITU_LIGHT2].from_file(renderer, _data_path + "lakitu/2.png", lakitu_width);
    ok = ok && _lakitu_status_imgs[LAKITU_LIGHT3].from_file(renderer, _data_path + "lakitu/3.png", lakitu_width);
    ok = ok && _lakitu_status_imgs[LAKITU_RACE_OVER].from_file(renderer, _data_path + "lakitu/finish.png", lakitu_width);

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
    if (_music)
      Mix_FreeMusic( _music );
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool restart_race() {
    DEBUG_PRINT("restart_race()\n");
    Mix_HaltMusic();
    _game_status = GAME_STATUS_WAITING;
    _lakitu_status = LAKITU_INVISIBLE;
    _lakitu.set_angle(0);
    _last_lap_played = false;
    reset_players_curses_items();
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  bool finish_race() {
    DEBUG_PRINT("Status: GAME_STATUS_RACE_OVER\n");
    _game_status = GAME_STATUS_RACE_OVER;
    Mix_HaltMusic();
    bool ok = play_sound("you-win.wav");
    _countdown.reset();
    _lakitu_status = LAKITU_RACE_OVER;
    _lakitu.set_texture(&_lakitu_status_imgs[_lakitu_status]);
    reset_players_curses_items();
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

  bool render() {
    DEBUG_PRINT("render()-%g s!\n", _race_timer.getTimeSeconds());
    SDL_SetRenderDrawColor( renderer, 150, 150, 255, 255 ); // light blue background
    SDL_RenderClear( renderer );

    bool ok = true;
    // render each player
    for (unsigned int i = 0; i < _nplayers; ++i)
      ok = ok && _players[i].render(renderer);

    if (_game_status == GAME_STATUS_COUNTDOWN) {
      ok = ok && _lakitu.render(renderer);

    } // end GAME_STATUS_COUNTDOWN

    if (_game_status == GAME_STATUS_RACE) {
      if (_lakitu_status == LAKITU_LIGHT3) // show lakitu going upwards
        ok = ok && _lakitu.render(renderer);
    } // end GAME_STATUS_RACE

    if (_game_status == GAME_STATUS_RACE_OVER) {
      ok = ok && _lakitu.render(renderer);
    } // end GAME_STATUS_RACE_OVER

    SDL_RenderPresent( renderer);
    return true;
  }

protected:

  //////////////////////////////////////////////////////////////////////////////
  // TODO
  //! check if each player has a subscriber for camera
  //! \return true if everything OK, false otherwise.
  //! If false, the background image corresponding to the robots needs to be displayed

  //////////////////////////////////////////////////////////////////////////////

  //! check if each player has a publisher for joypad and a subscriber for cmd_vel
  //! \return true if everything OK, false otherwise.
  //! If false, the image needs to be displayed
  bool check_joypads_robots() {
    bool checkok = true;
    for (unsigned int player_idx = 0; player_idx < _nplayers; ++player_idx) {
      Player* p = &(_players[player_idx]);
      // sanity checks: check joypad status
      if (p->_joypad_status != JOYPAD_OK)
        checkok = false;
      else if (p->_last_joy_updated.getTimeSeconds() > .5) {
        ROS_WARN("Player %i: JOYPAD_TIMEOUT", player_idx);
        p->_joypad_status = JOYPAD_TIMEOUT;
        checkok = false;
      }
      // sanity checks: check robot status
      if (p->_cmd_vel_pub.getNumSubscribers())
        p->_robot_status = ROBOT_OK;
      else {
        ROS_WARN("Player %i: ROBOT_TIMEOUT", player_idx);
        p->_robot_status = ROBOT_TIMEOUT;
        checkok = false;
      }
    } // end for _player_idx
    return checkok;
  } // end check_joypads_robots()

  //////////////////////////////////////////////////////////////////////////////

  bool update_waiting() {
    bool checkok = check_joypads_robots();
    // check state changes
    if (checkok) { // start countdown
      DEBUG_PRINT("Status: GAME_STATUS_COUNTDOWN\n");
      play_sound("begin-race.wav");
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
    check_joypads_robots();
    double time = _countdown.getTimeSeconds();
    // check state changes
    if (time >= 3 + 2.40 && _lakitu_status == LAKITU_LIGHT2) { // start race
      _race_timer.reset();
      _lakitu_status = LAKITU_LIGHT3;
      _lakitu.set_texture(&_lakitu_status_imgs[_lakitu_status]);
      _game_status = GAME_STATUS_RACE;
      Mix_PlayMusic( _music, -1 );
      // play a mushroom sound if needed
      bool play_dud = false, play_rocket = false;
      for (unsigned int i = 0; i < _nplayers; ++i) {
        if (_players[i]._curse == CURSE_DUD_START) // refresh dud start
          _players[i].receive_curse(CURSE_DUD_START, i);
        play_dud = play_dud || (_players[i]._curse == CURSE_DUD_START);
        play_rocket = play_rocket || (_players[i]._curse == CURSE_ROCKET_START);
      }
      if (play_rocket) play_sound("boost.wav");
      if (play_dud) play_sound("spinout.wav");
      DEBUG_PRINT("Status: GAME_STATUS_RACE\n");
    }
    else if (time >= 3 + 1.20 && _lakitu_status == LAKITU_LIGHT1) {
      _lakitu_status = LAKITU_LIGHT2;
      _lakitu.set_texture(&_lakitu_status_imgs[_lakitu_status]);
    }
    else if (time >= 3 && _lakitu_status == LAKITU_LIGHT0) {
      _lakitu_status = LAKITU_LIGHT1;
      _lakitu.set_texture(&_lakitu_status_imgs[_lakitu_status]);
      _lakitu.set_position(_lakitu_center); // just in case
      play_sound("racestart.wav");
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
      play_sound("last-lap.wav");
    }

    // check joypads
    if (!check_joypads_robots())
      return true;

    // check items
    if (_last_roulette.getTimeSeconds() > _min_time_roulette && rand() % 100 >= 90)
      set_players_roulette();

    // check if each player item or curse is over
    for (unsigned int player_idx = 0; player_idx < _nplayers; ++player_idx) {
      Player* p = &(_players[player_idx]);
      // check items
      if (p->_item == ITEM_ROULETTE) {
        if (p->_roulette_timer.getTimeSeconds() > 3) // roulette timeout
          p->item_button_cb(true);
        // rewind sound
        else if (_last_roulette_sound_play.getTimeSeconds() > .782) { // 0.782 seconds
          play_sound("itemreel.wav");
          _last_roulette_sound_play.reset();
        }
      } // end ITEM_ROULETTE

      // clean curses if needed
      double time_curse = p->_curse_timer.getTimeSeconds();
      if (p->_curse == CURSE_REDSHELL_COMING
          && time_curse > .1
          && (time_curse > _curse_timeout[CURSE_REDSHELL_COMING]
              || rand() % 50 == 0)) { // random end time
        play_sound("cpuspin.wav");
        p->play_animation("hit");
        p->play_animation_caster(player_idx, _players, "mock");
        p->receive_curse(CURSE_REDSHELL_HIT, p->_curse_caster_idx);
      }
      else if (p->_curse == CURSE_TIMEBOMB_COUNTDOWN
               && _timebomb.getTimeSeconds() > 4.935) { // 4.935 seconds
        p->play_animation("hit");
        p->play_animation_caster(player_idx, _players, "mock");
        p->receive_curse(CURSE_TIMEBOMB_HIT, p->_curse_caster_idx);
      }
      else if ((p->_curse != CURSE_NONE) // whatever curse
               && time_curse > _curse_timeout[p->_curse]) {
        DEBUG_PRINT("Player %i: timeout on curse %i\n", player_idx,_curse);
        p->receive_curse(CURSE_NONE, player_idx);
      }
    } // end for _player_idx
    return true;
  } // end update_race()

  //////////////////////////////////////////////////////////////////////////////

  bool update_race_over() {
    check_joypads_robots();
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
  }

  //////////////////////////////////////////////////////////////////////////////

  //!\return true if render OK or already done // This functionnality has been
  //!\ remove in order to have double color timer (for visibility)
  bool render_countdown2texture(const int time, int r, int g, int b) {
    //~ if (_last_renderer_countdown_time == time) ;
    //~ return true;
    std::ostringstream time_str; time_str << time;
    _last_renderer_countdown_time = time;
    return _countdown_texture.loadFromRenderedText(renderer, _countdown_font, time_str.str(),
                                                   r, g, b);
  }

  //////////////////////////////////////////////////////////////////////////////

  bool set_players_roulette() {
    _last_roulette.reset();
    // set roulette to everybody
    for (unsigned int i = 0; i < _nplayers; ++i) {
      Player* p = &(_players[i]);
      // check player has no item or curse
      if (p->_item != ITEM_NONE || p->_curse != CURSE_NONE)
        continue;
      p->receive_item(ITEM_ROULETTE);
      p->_roulette_timer.reset();
    }
    return true;
  }

  //////////////////////////////////////////////////////////////////////////////

  void reset_players_curses_items() {
    for (unsigned int i = 0; i < _nplayers; ++i) {
      _players[i]._curse = CURSE_NONE;
      _players[i]._item= ITEM_NONE;
    }
  }

  //////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////

  struct Player {
    Player(Game* game, unsigned int player_idx) { // ctor
      _game = game;
      _player_idx = player_idx;
      _item = ITEM_NONE;
      _curse = CURSE_NONE;
      _joypad_status = JOYPAD_NEVER_RECEIVED;
      _robot_status = ROBOT_NEVER_RECEIVED;
      _camera_status = CAMERA_NEVER_RECEIVED ;
      _scale_linear = _scale_angular = 1;
      _item_button_before = _sharp_turn_before = false;
      // determine background color
      switch (player_idx % 4) {
        case 0:
          _bgcolor = cv::Vec4i(121, 28, 248, 255); break;
        case 1:
          _bgcolor = cv::Vec4i(22, 124, 78, 255); break;
        case 2:
          _bgcolor = cv::Vec4i(233, 109, 20, 255); break;
        case 3:
        default:
          break;
      }
    }

    ////////////////////////////////////////////////////////////////////////////

    void create_pub_sub() {
      // joy params
      ros::NodeHandle nh_private("~");
      nh_private.param(_name + "/axis_angular",_axis_angular, 0);
      nh_private.param(_name + "/axis_linear",_axis_linear, 1);
      nh_private.param(_name + "/axis_90turn",_axis_90turn, 4);
      nh_private.param(_name + "/axis_180turn",_axis_180turn, 5);
      nh_private.param(_name + "/button_item",_button_item, 7);
      nh_private.param(_name + "/scale_angular",_scale_angular, 1.0);
      nh_private.param(_name + "/scale_linear",_scale_linear, 1.0);

      // create subscribers
      _joy_sub = _game->_nh_public.subscribe<sensor_msgs::Joy>
          (_name + "/joy", 1, &Player::joy_cb, this);
      bool use_rgb = true;
      nh_private.param("use_rgb",use_rgb, use_rgb);
      if (use_rgb)
        _rgb_sub = _game->_it.subscribe
            (_name + "/image", 1, &Player::rgb_cb, this);

      // create publishers
      _cmd_vel_pub  = _game->_nh_public.advertise<geometry_msgs::Twist>
          (_name + "/cmd_vel", 1);
      _animation_pub  = _game->_nh_public.advertise<std_msgs::String>
          (_name + "/animation", 1);
      _sharp_turn_pub = _game->_nh_public.advertise<std_msgs::Float32>
          (_name + "/sharp_turn", 1);

      _item_size_updated = false;
    }

    ////////////////////////////////////////////////////////////////////////////

    inline unsigned int nplayers() const { return _game->_nplayers; }

    ////////////////////////////////////////////////////////////////////////////

    bool has_rgb() const { return !_rgb.empty(); }

    ////////////////////////////////////////////////////////////////////////////

    void receive_item(Item i) { _item = i; }

    ////////////////////////////////////////////////////////////////////////////

    bool render(SDL_Renderer* renderer) {
      // draw colored background
      SDL_SetRenderDrawColor( renderer,
                              _bgcolor[0], _bgcolor[1], _bgcolor[2], _bgcolor[3]);
      SDL_Rect rect;
      rect.x = _tl_win.x;
      rect.y = _tl_win.y;
      rect.w = _game->player_w;
      rect.h = _game->player_h;
      SDL_RenderFillRect(renderer, &rect);

      // draw image camera if available
      bool ok = true;
      if (has_rgb()){
        ok = ok && _rgb.render(renderer, _tl_camview);}
      else{
        ok = ok && _avatar.render(renderer,
                                  Point2d(_tl_win.x + 0.25*_game->player_w,
                                          _tl_win.y + 0.25*_game->player_h));
      }

      // draw boundary lines between players
      SDL_SetRenderDrawColor( renderer, 0, 0, 0, 255 );
      SDL_RenderDrawRect(renderer,&rect);

      // draw joypad status
      if (_joypad_status != JOYPAD_OK)
        ok = ok && _game->_joypad_status_imgs[_joypad_status].render(renderer,
                                                                     _item_tl_corner);
      // draw robot status
      else if (_robot_status != ROBOT_OK)
        ok = ok && _game->_robot_status_imgs[_robot_status ].render(renderer,
                                                                    _item_tl_corner);
      if (_game->_game_status != GAME_STATUS_RACE)
        return ok;

      ok = ok && _game->_bg_imgs[BG_ITEMS].render(renderer, Point2d(_item_tl_corner.x-5,_item_tl_corner.y-5)); // Item Background
      //ROS_WARN("Redraw player %i!", i);
      // do nothing if no joypad or robot
      if (_joypad_status != JOYPAD_OK || _robot_status != ROBOT_OK)
        return ok;

      if (_curse != CURSE_NONE)
        ok = ok && _game->_curse_imgs[(int)_curse].render(renderer,_curse_tl);
      if (_item == ITEM_ROULETTE)
        ok = ok && _game->_item_imgs[random_item()].render(renderer,_item_tl_corner);
      else if (_item != ITEM_NONE)
        ok = ok && _game->_item_imgs[_item].render(renderer,_item_tl_corner);

      int time = _game->_race_duration + 1 - _game->_race_timer.getTimeSeconds();
      ok = ok && _game->render_countdown2texture(time, 0, 0, 0) // black
          && _game->_countdown_texture.render_center
          (renderer, Point2d(_tl_win.x + _game->player_w/5, _tl_win.y+ _game->player_h/6),0.4)
          && _game->render_countdown2texture(time, 255, 10, 10) // red
          && _game->_countdown_texture.render_center
          (renderer, Point2d(_tl_win.x + _game->player_w/5+2, _tl_win.y+ _game->player_h/6+2),0.4);
      // && _countdown_texture.render_center(renderer, Point2d(_winw/2,_winh/2),0.8);
      return ok;
    } // end render()

    ////////////////////////////////////////////////////////////////////////////

    void receive_curse(Curse c, unsigned int curse_caster_) {
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

    void play_animation_caster(unsigned int self_idx,
                               std::vector<Player> & players,
                               const std::string & s) {
      if (self_idx == _curse_caster_idx)
        return;
      players[_curse_caster_idx].play_animation(s);
    }

    ////////////////////////////////////////////////////////////////////////////

    bool sharp_turn_button_cb(double angle_rad,
                              bool skip_repetition_check = false) {
      bool retval = false;
      if (fabs(angle_rad) > 1E-2
          && (skip_repetition_check || fabs(angle_rad -_sharp_turn_before)>1E-2)) {
        std_msgs::Float32 msg;
        msg.data = angle_rad;
        _sharp_turn_pub.publish(msg);
        DEBUG_PRINT("Player %i: sharp turn of angle %g rad!\n", _player_idx, angle_rad);
        retval = true;
      }
      _sharp_turn_before = angle_rad;
      return retval;
    }

    ////////////////////////////////////////////////////////////////////////////

    bool item_button_cb(bool skip_repetition_check = false) {
      DEBUG_PRINT("item_button_cb(%i, %i)\n", _player_idx, skip_repetition_check);
      if (_game->_game_status != GAME_STATUS_RACE) // nothing to do
        return true;
      // check what to do if Item_button

      if (!skip_repetition_check &&_item_button_before)
        return false;
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
          _game->play_sound("menumove.wav");
          receive_curse(CURSE_NONE, _player_idx);
          target->receive_curse(CURSE_TIMEBOMB_COUNTDOWN, _player_idx);
        }
      } // end CURSE_TIMEBOMB_COUNTDOWN

      // check items
      if (pi == ITEM_BOO) { // swap items
        //play_sound("boo.wav");
        _game->play_sound("boosteal.wav");
        receive_item(ITEM_NONE);
        if (is_real_item(targeti)) {
          receive_item(targeti);
          target->receive_item(ITEM_NONE);
          target->receive_curse(CURSE_BOO, _player_idx);
        }
        else { // nothing to steal -> punish player!
          receive_curse(CURSE_BOO, _player_idx);
        }
      }
      else if (pi == ITEM_GOLDENMUSHROOM) {
        _game->play_sound("boost.wav");
        receive_item(ITEM_NONE);
        receive_curse(CURSE_GOLDENMUSHROOM, _player_idx);
      }
      else if (pi == ITEM_LIGHTNING) {
        _game->play_sound("lightning.wav");
        receive_item(ITEM_NONE);
        if (targetc != CURSE_STAR) { // do nothing if target has star
          target->receive_curse(CURSE_LIGHTNING, _player_idx);
          play_animation("mock");
        }
      }
      else if (pi == ITEM_MIRROR) {
        _game->play_sound("quartz.wav");
        receive_item(ITEM_NONE);
        if (targetc != CURSE_STAR) { // do nothing if target has star
          target->receive_curse(CURSE_MIRROR, _player_idx);
          play_animation("mock");
        }
      }
      else if (pi == ITEM_MUSHROOM) {
        _game->play_sound("boost.wav");
        receive_item(ITEM_NONE);
        receive_curse(CURSE_MUSHROOM, _player_idx);
      }
      else if (pi == ITEM_REDSHELL || pi == ITEM_REDSHELL2 || pi == ITEM_REDSHELL3) {
        _game->play_sound("cputhrow.wav");
        if (targetc != CURSE_STAR) // do nothing if target has star
          target->receive_curse(CURSE_REDSHELL_COMING, _player_idx);
        if (pi == ITEM_REDSHELL) // decrease red shell counter
          receive_item(ITEM_NONE);
        else if (pi == ITEM_REDSHELL2)
          receive_item(ITEM_REDSHELL);
        else if (pi == ITEM_REDSHELL3)
          receive_item(ITEM_REDSHELL2);
      }
      else if (pi == ITEM_STAR) {
        _game->play_sound("starman.wav");
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

      _item_button_before = true;
      return true;
    } // end item_button_cb()

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
          }
        } // end fabs(v) > .1
        v = w = 0; // can't move during countdown!
      } // end if GAME_STATUS_COUNTDOWN

      else if (_game->_game_status == GAME_STATUS_RACE) {
        if (c == CURSE_DUD_START) {
          v *= .2; // sloowww
          w += .3 *_scale_angular *
              cos(5*_game->_race_timer.getTimeSeconds()); // oscillate
        }
        else if (c == CURSE_GOLDENMUSHROOM)
          v *= 2; // 200% faster
        else if (c == CURSE_LIGHTNING) {
          //ROS_WARN("%i:CURSE_LIGHTNING!", _player_idx);
          v *= .2; // half speed
          w += .3 *_scale_angular *
              cos(5*_game->_race_timer.getTimeSeconds()); // oscillate
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
      _cmd_vel_pub.publish(vel);
      return true;
    } // end set_speed()

    ////////////////////////////////////////////////////////////////////////////

    void joy_cb(const sensor_msgs::Joy::ConstPtr& joy) {
      //DEBUG_PRINT("joy_cb(%i)\n", _player_idx);

      int naxes = joy->axes.size(), nbuttons = joy->buttons.size();
      if (naxes <_axis_90turn
          || naxes <_axis_180turn
          || naxes <_axis_linear
          || naxes <_axis_angular) {
        ROS_WARN_THROTTLE(1, "Only %i axes on joypad #%i!", naxes, _player_idx);
        _joypad_status = JOYPAD_BAD_AXES_NB;
        return;
      }
      if (nbuttons <_button_item) {
        ROS_WARN_THROTTLE(1, "Only %i buttons on joypad #%i!", nbuttons, _player_idx);
        _joypad_status = JOYPAD_BAD_BUTTONS_NB;
        return;
      }

      _joypad_status = JOYPAD_OK;
      _last_joy_updated.reset();
      // check sharp turns at 90° or 180°
      double angle = 0;
      if (fabs(joy->axes[_axis_90turn]) > 0.9)
        angle = (joy->axes[_axis_90turn] < 0 ? M_PI_2 : -M_PI_2);
      if (fabs(joy->axes[_axis_180turn]) > 0.9)
        angle = (joy->axes[_axis_180turn] < 0 ? 2 * M_PI : -M_PI);
      bool command_sent = sharp_turn_button_cb(angle, _player_idx);

      // check item button
      if (joy->buttons[_button_item])
        item_button_cb();
      else {
        _item_button_before = false;
      }

      // cheat: trigger ROULETTE with button 0
      //if (joy->buttons[0]) set_players_roulette();

      // if no command was sent till here: move robot with directions of axes
      if (command_sent)
        return;

      set_speed(joy->axes[_axis_linear] *_scale_linear,
                joy->axes[_axis_angular] *_scale_angular);
    } // end joy_cb();

    ////////////////////////////////////////////////////////////////////////////

    void rgb_cb(const sensor_msgs::ImageConstPtr& rgb) {

      if (!_rgb.from_ros_image(_game->renderer, *rgb,
                               _game->player_w, _game->player_h)) {
        ROS_WARN("Texture::from_ros_image() failed!");
        return;
      }

      //update of the player player_tl_camview in order to center the video image
      if (_game->player_w ==_rgb._width){ // Image has been scaled to max width

        _tl_camview.x =_tl_win.x;
        int paddingy = _game->player_h -_rgb._height;

        _tl_camview.y =_tl_win.y + paddingy/2;

      }else{ // Image has been scaled to max height

        _tl_camview.y =_tl_win.y;

        int paddingx = _game->player_w -_rgb._width;
        _tl_camview.x =_tl_win.x + paddingx/2;
      }

      //Also update of the player item_tl_corner and its size to fit the image

      //    if (!p->item_size_updated) {
      //        _item_w =_rgb._height/4;
      //        load_item(_item_w);

      //       item_tl_corner.x =player_tl_camview.x +_rgb._width -_item_w*1.2;
      //       item_tl_corner.y =player_tl_camview.y;
      //       curse_tl.x =item_tl_corner.x - 1.2*_item_w;
      //       curse_tl.y =item_tl_corner.y;

      //       item_size_updated=true; // Do the update only onte time by players.
      //    }
    }

    ////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////

    Game* _game;
    std::string _name;
    unsigned int _player_idx;
    Point2d _item_tl_corner, _curse_tl, _tl_win, _tl_camview;
    Item _item;
    Curse _curse;
    JoypadStatus _joypad_status;
    int _axis_linear, _axis_angular, _axis_90turn, _axis_180turn, _button_item;
    RobotStatus _robot_status;
    CameraStatus _camera_status;
    bool _sharp_turn_before, _item_button_before, _item_size_updated;
    double _scale_angular, _scale_linear;
    Timer _curse_timer, _roulette_timer, _last_joy_updated;
    unsigned int _curse_caster_idx;
    Texture _avatar, _rgb;
    cv::Vec4i _bgcolor;
    // ROS data
    ros::Subscriber _joy_sub ;
    image_transport::Subscriber _rgb_sub;
    ros::Publisher _cmd_vel_pub, _sharp_turn_pub, _animation_pub;
  }; // end struct Player //////////////////////////////////////////////////////

  SDL_Window* window;
  SDL_Renderer* renderer;
  int _winw, _winh;
  unsigned int _nplayers;
  std::vector<Player> _players;
  GameStatus _game_status;
  double _min_time_roulette, _timebomb_likelihood;
  Timer _last_roulette_sound_play, _timebomb, _last_roulette, _countdown, _race_timer;
  Timer::Time _race_duration;
  bool _last_lap_played;

  // ros stuff
  ros::NodeHandle _nh_public, _nh_private;
  //See later for dding subscription to image sent by camera using image_transport
  image_transport::ImageTransport _it;

  // sound stuff
  std::map<std::string, Mix_Chunk*> _chunks;
  Mix_Music *_music;

  // opencv stuff
  LakituStatus _lakitu_status;
  Point2d _lakitu_center ;
  Entity _lakitu;

  // time display stuff

  TTF_Font *_countdown_font;
  int _last_renderer_countdown_time;
  Texture _countdown_texture;

  // share textures
  std::vector<Texture> _lakitu_status_imgs, _item_imgs, _curse_imgs, _bg_imgs,
  _joypad_status_imgs, _robot_status_imgs;

  // items
  std::string _data_path, _sound_path;
  int _item_w;  // pixels
  int player_w, player_h; // size of player windows
  std::vector<double> _curse_timeout;

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
  ros::Rate update_rate(25);
  Timer last_render;

  while (ros::ok()) {
    if (!game.update()) {
      ROS_ERROR("game.update() failed!");
      return false;
    }
    if (last_render.getTimeSeconds() > 0.06) { // 15 Hz
      if (!game.render()) {
        ROS_ERROR("game.render() failed!");
        return false;
      }
      last_render.reset();
    }

    ros::spinOnce();

    if (!update_rate.sleep()){
      // ROS_WARN(" Main Cycle Rate non respected");
    }

  } // end while (ros::ok())

  return (game.clean() ? 0 : -1);
}
