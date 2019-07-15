#ifndef _NUBOT_BEHAVIOUR_H
#define _NUBOT_BEHAVIOUR_H

#include "subtargets.h"
#include "define.hpp"

#define NB
#define NM

using namespace std;
namespace nubot{
class Behaviour
{

public:
    Behaviour(std::vector<DPoint> &obstacles);
    ~ Behaviour();

    float basicPDControl(float pgain,float dgain, float err,float err1, float maxval);
    void move2Position(float pval, float dval, DPoint target, float maxvel,
                       const DPoint  & _robot_pos,const Angle  & _robot_ori );
    void move2target(float pval, float dval,DPoint target, DPoint realtarvel, float maxvel,
                     const DPoint  & _robot_pos,const Angle  & _robot_ori);
    void move2Positionwithobs(float pval, float dval, DPoint target, float maxvel,
                              const DPoint & _robot_pos, const Angle & _robot_ori, bool avoid_ball=true);
    void revDecoupleFromVel(float vx,float vy,float &positivemax_rev,float &negativemax_rev);
    /** rotate to the target orientation by using PD control*/
    void rotate2AbsOrienation(float pval, float dval, float orientation,float maxw,const Angle & _robot_ori);
    void clearBehaviorState();
    void setAppvx(double vx);
    void setAppvy(double vy);
    void setAppw(double w);
    void accelerateLimit(const double &_acc_thresh = 25, const bool & use_convected_acc = true);
    void clear();
public:
    float app_vx_;
    float app_vy_;
    float app_w_;
    float last_app_vx_;
    float last_app_vy_;
    float last_app_w_;
    float last_speed;

    Subtargets m_subtargets_;
 };
}

#endif // _NUBOT_BEHAVIOUR_H
