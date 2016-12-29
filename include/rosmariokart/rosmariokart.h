#ifndef ROSMARIOKART_H
#define ROSMARIOKART_H

#include <stdlib.h>

enum GameStatus {
  GAME_STATUS_WAITING   = 0, // waiting for robots or cmd_vels
  GAME_STATUS_COUNTDOWN = 1, // countdown + lakitu
  GAME_STATUS_RACE      = 2, // race time
  GAME_STATUS_RACE_OVER = 3, // players cant move anymore
  NGAME_STATUES         = 4
};

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
  NITEMS               = 11
};

enum Background {
  BG_NONE              = 0,
  BG_ITEMS             = 1,
  BG_GO                = 2,
  NBG                  = 3
};

enum Curse {
  CURSE_NONE               = 0,
  CURSE_BOO                = 1,
  CURSE_DUD_START          = 2, // missed rocket start - http://www.mariowiki.com/Rocket_Start
  CURSE_GOLDENMUSHROOM     = 3,
  CURSE_LIGHTNING          = 4,
  CURSE_MIRROR             = 5,
  CURSE_MUSHROOM           = 6,
  CURSE_REDSHELL_COMING    = 7,
  CURSE_REDSHELL_HIT       = 8,
  CURSE_ROCKET_START       = 9, // fast start - http://www.mariowiki.com/Rocket_Start
  CURSE_STAR               = 10,
  CURSE_TIMEBOMB_COUNTDOWN = 11,
  CURSE_TIMEBOMB_HIT       = 12,
  NCURSES                  = 13
};

enum CmdVelStatus {
  CMD_VEL_NEVER_RECEIVED = 0,
  CMD_VEL_OK             = 1,
  CMD_VEL_TIMEOUT        = 4,
  NCMD_VEL_STATUSES      = 5
};

enum RobotStatus {
  ROBOT_NEVER_RECEIVED = 0,
  ROBOT_OK             = 1,
  ROBOT_TIMEOUT        = 2,
  NROBOT_STATUSES      = 3
};

enum LakituStatus {
  LAKITU_INVISIBLE      = 0,
  LAKITU_LIGHT0         = 1,
  LAKITU_LIGHT1         = 2,
  LAKITU_LIGHT2         = 3,
  LAKITU_LIGHT3         = 4,
  LAKITU_RACE_OVER      = 5,
  NLAKITU_STATUSES      = 6
};

////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

inline bool is_real_item(Item i) {
  return (i != ITEM_NONE && i != ITEM_ROULETTE);
}

////////////////////////////////////////////////////////////////////////////////

inline Item random_item() {
  //return ITEM_MUSHROOM; // debug test
  Item i = (Item) (rand() % NITEMS);
  return (is_real_item(i) ? i : random_item());
}

#endif // ROSMARIOKART_H
