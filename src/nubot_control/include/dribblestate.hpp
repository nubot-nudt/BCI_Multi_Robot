#ifndef DRIBBLE_STATUS_HPP
#define DRIBBLE_STATUS_HPP

#include "core.hpp"

class DribbleState
{
public:
  bool is_dribble_;
  int  time_count_;
  nubot::DPoint satrt_point_;

  DribbleState()
      :is_dribble_(false),time_count_(0)
  {
  }

  void reset()
  {
      is_dribble_ = false;
  }

  void set(const nubot::DPoint &_pos_satrt)
  {
      is_dribble_ = true;
      satrt_point_ = _pos_satrt;
  }
  void update( const bool &_is_dribble, const nubot::DPoint &_pos_robot, const nubot::DPoint &_ball)
  {
      static int lost_cnt = 0;
      if( _is_dribble == is_dribble_ && _is_dribble )
          return;
      if( _is_dribble )
      {
          is_dribble_ = true;
          if(lost_cnt>=30)
              satrt_point_ = _pos_robot;
          lost_cnt = 0;
      }
      else
      {
          if( _ball.x_> 60 || fabs(_ball.y_)>30)
          {
              is_dribble_ = false;
              lost_cnt ++;
              if(lost_cnt>30)
                  lost_cnt = 30;
          }
      }
  }

  nubot::DPoint limitWithinCircle( const nubot::DPoint &_point_in, const int _radius = 300)
  {
      if( (_point_in-satrt_point_).norm()<_radius )
          return _point_in;
      else
          return satrt_point_ + (_point_in-satrt_point_) *( _radius/(_point_in-satrt_point_).norm() );
  }
};

#endif
