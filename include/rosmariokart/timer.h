/*!
  \file        timer.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2016/4/13

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
*/
#ifndef TIMER_H_
#define TIMER_H_

// c includes
#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

class Timer {
public:
  typedef double Time;
  static const Time NOTIME = -1;
  Timer() { reset(); }
  virtual inline void reset() {
    gettimeofday(&start, NULL);
  }
  //! get the time since ctor or last reset (milliseconds)
  virtual inline Time getTimeSeconds() const {
    struct timeval end;
    gettimeofday(&end, NULL);
    return (Time) (// seconds
                   (end.tv_sec - start.tv_sec)
                   +
                   // useconds
                   (end.tv_usec - start.tv_usec)
                   / 1E6);
  }
private:
  struct timeval start;
}; // end class Timer

////////////////////////////////////////////////////////////////////////////////

class Rate {
public:
  Rate(double rate_hz) : _rate_hz(rate_hz) {
    _period_sec = 1. / _rate_hz;
  }

  double sleep() {
    double time_left = _period_sec - _timer.getTimeSeconds();
    if (time_left > 1E-3) // 1 ms
      usleep(1E6 * time_left);
    _timer.reset();
    return time_left;
  }

private:
  Timer _timer;
  double _rate_hz, _period_sec;
}; // end class Rate

#endif /*TIMER_H_*/

