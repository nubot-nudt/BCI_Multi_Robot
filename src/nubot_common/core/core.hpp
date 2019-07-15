#ifndef __NUBOT_CORE_HPP__
#define __NUBOT_CORE_HPP__

/// \brief compile the code with Gazebo 8.0, there are many warning, ignored them
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wzero-as-null-pointer-constant"
#pragma GCC diagnostic ignored "-Wconversion"
#pragma GCC diagnostic ignored "-Wdouble-promotion"

#include <time.h>
#include <fstream>
#include <string>
#include <iostream>
#include <stdio.h>
#include "ros/ros.h"
#include "Circle.hpp"
#include "Angle.hpp"
#include "DPoint.hpp"
#include "PPoint.hpp"
#include "Line.hpp"

#define CM2M_CONVERSION 0.01
#define M2CM_CONVERSION 100

const double WIDTHRATIO4FIELD = 1;
const double WIDTH_RATIO= 1 ;
const double ConstDribbleDisFirst  = 50;
const double ConstDribbleDisSecond = 40;
const double MAXVEL   =    300;
const double MAXW     =    5;
const double CHASSISRADIUS =  20.3;   //cm the radius of chassiss
const double REDUCTIONRATIO = 12.0;   //   reduction gear ratio
const double WHEELDIAMETER = 12.0;    //cm the diameter of wheel
const double RATE =  (REDUCTIONRATIO*60.0)/(SINGLEPI_CONSTANT*WHEELDIAMETER); // rate from 4 wheels linear velocity to rpm
const double LIMITEDRPM  =  12000.0;  // the top limited rpm
const double LIMITDRIBLLEDIS = 75.0;  // the limit distance to check dribble or not

//field size info
const int FIELD_CENTER_RADIUS = 200;
const int NOT_DATAUPDATE = 1200;
const int TEAM_NUM = 5;
const int OPP_TEAM_NUM = 4;
const int ROLENUM  = 7;
const int FIELD_LENGTH= 1800;
const int FIELD_WIDTH = 1200;
const double MAXDIS_FIELD = sqrt(FIELD_LENGTH*FIELD_LENGTH+FIELD_WIDTH*FIELD_WIDTH);

const int FIELD_XLINE1 = 900;
const int FIELD_XLINE2 = 825;
const int FIELD_XLINE3 = 675;
const int FIELD_XLINE4 = 0;
const int FIELD_XLINE5 = -675;
const int FIELD_XLINE6 = -825;
const int FIELD_XLINE7 = -900;

const int FIELD_YLINE1 =  600;
const int FIELD_YLINE2 =  325;
const int FIELD_YLINE3 =  175;
const int FIELD_YLINE4 =  -175;
const int FIELD_YLINE5 =  -325;
const int FIELD_YLINE6 =  -600;

const int FIELD_POST_RADIUS = 80;
const int LOCATIONERROR = 30;
const int ANGLEERROR = 5;

//time set
const int P300_TIMER    = 10;
const int RESET_TIMER   = 200;
const int PREDICT_TIMER = 2;

//payoff matrix
const int PAY_OFF[3][3] ={{1,3,5},{3,3,3},{5,3,1}};

enum Goal{GOAL_UPPER    = 0,
          GOAL_MIDUPPER = 1,
          GOAL_MIDDLE   = 2,
          GOAL_MIDLOWER = 3,
          GOAL_LOWER    = 4
};

enum BallStates{FREE_BALL   = 0,
                OUR_DRIBBLE = 1,
                UNKNOWN     = 2,

};

enum Buttons{SELECTION_ONE   = 0,
             SELECTION_TWO   = 1,
             SELECTION_THREE = 2,
             SELECTION_FOUR  = 3,
             SELECTION_FIVE  = 4,
             SELECTION_SIX   = 5
            };

enum Roles{NOROLE     = 0,
           ACTIVE     = 1,
           PASSIVE    = 2,
           ASSISTANT  = 3,
           MIDFIELD   = 4,
           SUBSTITUTE = 5
          };

enum Actions{No_Action         =0,
             Stucked           =1,
             CanNotSeeBall     =2,
             CatchBall         =3,
             AvoidObs          =4,
             Pass              =5,
             Receive           =6,
             Shoot             =7,
             Positioned        =8,
            };

enum Layers{STRATEGY_L = 0,
            ATTACK_L   = 1,
            DEFEND_L   = 3,
            NUBOT_L    = 4,
            ACTION_L   = 5
           };

enum Strategy{NO_STRATEGY     = 0,

              STRATEGY_ATTACK = 1,
              STRATEGY_DEFEND = 2,
              STRATEGY_ROBOT  = 3,

              CONSERVATIVE = 4,
              BALANCE      = 5,
              RADICAL      = 6,

              REGIONAL = 7,
              MAN2MAN  = 8,
              FOCUS    = 9,

              NUBOT_ONE   = 10,
              NUBOT_TWO   = 11,
              NUBOT_THREE = 12,
              NUBOT_FOUR  = 13,
              NUBOT_FIVE  = 14,

              UP_MAXSPEED   = 15,
              DOWN_MAXSPEED = 16,
              CHANGE_ROBOT  = 17,

              RETURN        = 18
             };

enum RobotControl{STOPROBOT   = 0,
                  STARTROBOT  = 1
};

#endif 
