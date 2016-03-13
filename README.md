# *rosmariokart*

A "mariokart" clone, but made for real robots.
<p align="center">
<img src="https://raw.githubusercontent.com/arnaud-ramey/rossumo/master/doc/jumping_sumo.jpg"
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

- `curse_XXX_timeout`
  [double, seconds, default: 2 seconds]

  The duration of each curse.
  `XXX` is among the following:
  `boo, goldenmushroom, lightning, mirror, mushroom, redshell_coming, redshell_hit, star, timebomb_hit`.

- `axis_180turn`
  [int, default: 4]

  The joystick axis to perform 180° turns.

- `axis_90turn`
  [int, default: 3]

  The joystick axis to perform 90° turns.

- `axis_angular`
  [int, default: 2]

  The joystick axis to control angular speed.

- `axis_linear`
  [int, default: 2]

  The joystick axis to control linear speed.

- `button_item`
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

- `cmd_vel`
[geometry_msgs::Twist, (m/s, rad/s)]

The instantaneous speed order.

Publications
------------

- `/ROBOT/cmd_vel`, for ex: `/mip/scale_linar`
  [geometry_msgs::Twist]
  The speed orders.

- `/ROBOT/animation`, for ex: `/mip/animation`
  [std_msgs::String]
  Predefined animations that can be specific to each robot.
  Among `win, lose, hit, hit2, mock`.

- `/ROBOT/sharp_turn`, for ex: `/mip/sharp_turn`
  [std_msgs::Float32]
  The speed orders.

Credits
=======

Images:

  - `TimeBombCurse.png`
    [pixabay.com](https://pixabay.com/en/explosion-detonation-blast-burst-155624/)
  - `BooCurse.png`
    [nocookie.net](http://fantendo.wikia.com/wiki/Boo_%28species%29)
  - `TimeBomb.png`
    [isthisabomb.com](http://isthisabomb.com/img/18.png)
  - `jumping_sumo_brown.jpg`
    [amain.com](http://images.amain.com/images/large/pta/ptapf724002.jpg)
  - `ori-robot-mip-noir-wowwee-1280.jpg`
    [robot-advance.com](http://www.robot-advance.com/ori-robot-mip-noir-wowwee-1280.jpg)

Sounds:

  - `boo.wav, starman.wav` :
    [superluigibros.com](http://www.superluigibros.com/mario-kart-64-sound-effects-wav)
  - `boost.wav, boosteal.wav, cpuspin.wav, cputhrow.wav, gotitem.wav, itemreel.wav, racestart.wav` :
    [superluigibros.com](http://www.superluigibros.com/super-mario-kart-sound-effects-wav)
  - `mk64_countdown.wav` :
    [themushroomkingdom.net](http://themushroomkingdom.net/media/mk64/wav)
  - `lightning.wav` :
    [youtube.com](https://www.youtube.com/watch?v=IXUoY_KgCko)
  - `quartz.wav` :
    [freesound.org](https://www.freesound.org/people/quartzgate/sounds/177868/)
  - `timebomb.wav` :
    [soundbible.com](http://soundbible.com/1203-Time-Bomb.html)


Tracks:
- [mariokart.wikia.com](http://mariokart.wikia.com/wiki/Mario_Circuit_1)

m8rkq88vl
m8r-kq88vl@mailinator.com
