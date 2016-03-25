# *rosmariokart*

A "mariokart" clone, but made for real robots.
<p align="center">
<img src="https://raw.githubusercontent.com/arnaud-ramey/rosmariokart/master/doc/logo.png"
   alt="sumo" style="width: 300px"/>
</p>


ROS driver node
===============

To launch the game:

```bash
$ roslaunch rosmariokart rosmariokart.launch
```

Node parameters
---------------

- `~player_1_name, ~player_2_name, ~player_3_name, ~player_4_name`
  [string, default: ""]

  The name of each player. Leave empty for no player.
  At least fields `player_1_name` and `player_2_name` must be set.
  As documented below, these names will be used as namespaces to access
  various topics and parameters.

- `~curse_XXX_timeout`
  [double, seconds, default: 2 seconds]

  The duration of each curse.
  `XXX` is among the following:
  `boo, goldenmushroom, lightning, mirror, mushroom, redshell_coming, redshell_hit, star, timebomb_hit`.

- `~axis_180turn`
  [int, default: 4]

  The joystick axis to perform 180° turns.

- `~axis_90turn`
  [int, default: 3]

  The joystick axis to perform 90° turns.

- `~axis_angular`
  [int, default: 2]

  The joystick axis to control angular speed.

- `~axis_linear`
  [int, default: 2]

  The joystick axis to control linear speed.

- `~button_item`
  [int, default: 3]

  The joystick button for throwing an item.

- `/ROBOT/scale_angular`, for ex: `/mip/scale_angular`
  [int, rad.s-1/joy_unit, default: 1]

  The multiplication factor that converts an axis value into an angular speed.

- `/ROBOT/scale_linear`, for ex: `/mip/scale_linar`
  [int, rad.s-1/joy_unit, default: 1]

  The multiplication factor that converts an axis value into a linear speed.

Subscriptions
-------------

- `/ROBOT/joy`, for ex: `/mip/joy`
  [sensor_msgs::Joy]

  The joystick commands.

Publications
------------

- `/ROBOT/cmd_vel`, for ex: `/mip/scale_linar`
  [geometry_msgs::Twist]

  The speed orders.

- `/ROBOT/animation`, for ex: `/mip/animation`
  [std_msgs::String]

  Predefined animations that can be specific to each robot.
  Among `hit, lose, mock, win`.

- `/ROBOT/sharp_turn`, for ex: `/mip/sharp_turn`
  [std_msgs::Float32]

  The on-the-spot speed orders, for sharp turns like 90°.

Credits
=======

Images
------

  - `doc/ori-robot-mip-noir-wowwee-1280.jpg`:
    [robot-advance.com](http://www.robot-advance.com/ori-robot-mip-noir-wowwee-1280.jpg)
  - `items/BooCurse.png`:
    [nocookie.net](http://fantendo.wikia.com/wiki/Boo_%28species%29)
  - `items/jumping_sumo_brown.jpg`:
    [amain.com](http://images.amain.com/images/large/pta/ptapf724002.jpg)
  - `items/joypad.png`:
    [myiconfinder.com](http://www.myiconfinder.com/icon/console-control-game-games-joy-joystick-manipulator-joypad-pad-joy-color-4-flat-metro-ui-dock/2533)
  - `items/TimeBomb.png`:
    [isthisabomb.com](http://isthisabomb.com/img/18.png)
  - `items/TimeBombCurse.png`:
    [pixabay.com](https://pixabay.com/en/explosion-detonation-blast-burst-155624/)
  - `robots/random_robot.png`:
    [openclipart.org](https://openclipart.org/detail/170101/cartoon-robot)
  - `robots/stage_black_bg.png`:
    [playerstage.org](http://playerstage.sourceforge.net/doc/stage-svn/index.html)
  - `robots/white_mip_black_bg.png`:
    [wowwee.com](http://store.wowwee.com/images/products/personalizations/2049.jpg)
  - `robots/white_sumo_black_bg.png`:
    [parrot.com](http://www.parrot.com/media/slideshows/slides/2015/01/26/165619437566.jpg)

Sounds
------

  - `starman.wav`:
    [superluigibros.com](http://www.superluigibros.com/mario-kart-64-sound-effects-wav)
  - `boost.wav, boosteal.wav, cpuspin.wav, cputhrow.wav, gotitem.wav, itemreel.wav, racestart.wav`:
    [superluigibros.com](http://www.superluigibros.com/super-mario-kart-sound-effects-wav)
  - `mk64_countdown.wav`:
    [themushroomkingdom.net](http://themushroomkingdom.net/media/mk64/wav)
  - `lightning.wav`:
    [youtube.com](https://www.youtube.com/watch?v=IXUoY_KgCko)
  - `quartz.wav`:
    [freesound.org](https://www.freesound.org/people/quartzgate/sounds/177868/)
  - `timebomb.wav`:
    [soundbible.com](http://soundbible.com/1203-Time-Bomb.html)


Tracks
------

  - `config/bitmaps/Mariocircuit1.*`
    [mariokart.wikia.com](http://mariokart.wikia.com/wiki/Mario_Circuit_1)

m8rkq88vl
m8r-kq88vl@mailinator.com
