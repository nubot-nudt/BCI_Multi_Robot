#include "behaviour.hpp"

using namespace nubot;
Behaviour::Behaviour(std::vector<DPoint> &obstacles)
{
    app_vx_ = 0;
    app_vy_ = 0;
    app_w_  = 0;
    last_app_vx_ = 0;
    last_app_vy_ = 0;
    last_app_w_ = 0;
    m_subtargets_.Obstacles_=& obstacles;
}

Behaviour::~Behaviour()
{
}
void Behaviour::clear()
{
    app_vx_ = 0;
    app_vy_ = 0;
    app_w_  = 0;
    last_app_vx_ = 0;
    last_app_vy_ = 0;
    last_app_w_ = 0;
}

void
Behaviour::revDecoupleFromVel(float vx,float vy,float &positivemax_rev,float &negativemax_rev)
{

    float a =vx +  vy;

    float rate =  RATE;


    float positivewmax,negativewmax;

    positivewmax = (2*LIMITEDRPM-1.414*rate*a)/(2*rate*CHASSISRADIUS);
    negativewmax = (-2*LIMITEDRPM-1.414*rate*a)/(2*rate*CHASSISRADIUS);


    positivemax_rev = positivewmax;
    negativemax_rev = negativewmax;

}
float
Behaviour::basicPDControl(float pgain,float dgain, float err,float err1, float maxval)
{
    float _e1 = err1;
    float kp =  pgain;
    float kd=  dgain;
    float _e=err;
    float retval = 0;
    retval  = kp *_e +kd*(_e -_e1) ;
    if(fabs(retval) > maxval)
    {
        if(retval>0) retval= maxval;
        else    retval=-maxval;
    }
    return retval;
}

// tranlation control move to the target point by using PD control
void
Behaviour::move2Position(float pval, float dval, DPoint target, float maxvel,
                         const DPoint & _robot_pos,const Angle & _robot_ori )
{

    float _pos_e = _robot_pos.distance(target);
    DPoint relposoftarget =  target  - _robot_pos;
    float tar_theta = relposoftarget.angle().radian_;
    static float _pos_e1 = 0;
    float speed  = 0;
    speed = basicPDControl(pval,dval,_pos_e,_pos_e1,maxvel);
    app_vx_ =  speed*cos(tar_theta - _robot_ori.radian_);
    app_vy_ =  speed*sin(tar_theta - _robot_ori.radian_);
    double v=sqrt(app_vx_*app_vx_+app_vy_*app_vy_);

    if(v>maxvel)
    {
        app_vx_=app_vx_*maxvel/v;
        app_vy_=app_vy_*maxvel/v;
    }
    _pos_e1  = _pos_e;
}

void
Behaviour::move2target(float pval, float dval,DPoint target, DPoint realtarvel, float maxvel,
                       const DPoint & _robot_pos, const Angle & _robot_ori)
{
    float _pos_e = _robot_pos.distance(target);
    DPoint relposoftarget =  target  - _robot_pos;
    float tar_theta = relposoftarget.angle().radian_;  //
    static float _pos_e1 = 0;
    float speed  = 0;
    speed = basicPDControl(pval,dval,_pos_e,_pos_e1,maxvel);
    app_vx_ =  speed*cos(tar_theta - _robot_ori.radian_) + realtarvel.x_;  //
    app_vy_ =  speed*sin(tar_theta - _robot_ori.radian_) + realtarvel.y_;

    double v=sqrt(app_vx_*app_vx_+app_vy_*app_vy_);
    if(v>maxvel)
    {
        app_vx_=app_vx_*maxvel/v;
        app_vy_=app_vy_*maxvel/v;
    }
    _pos_e1  = _pos_e;
}

void Behaviour::move2Positionwithobs(float pval, float dval, DPoint target, float maxvel,
                                     const DPoint & _robot_pos, const Angle & _robot_ori, bool avoid_ball)
{
    m_subtargets_.subtarget(target, _robot_pos, avoid_ball);
    double theta;

    if(m_subtargets_.subtargets_pos_==target)
    {
        move2Position(pval,dval,target,maxvel,_robot_pos,_robot_ori);
        theta = thetaof2p(_robot_pos,target);
    }
    else
        move2Position(pval,dval,m_subtargets_.subtargets_pos_,maxvel,_robot_pos,_robot_ori);
}

// rotation control

// 1   priory one ->   translation
// 2   priory two ->   orienatation

void
Behaviour::rotate2AbsOrienation(float pval, float dval, float orientation,float maxw,const Angle & _robot_ori)
{

    float theta_e = orientation -  _robot_ori.radian_;
    static float theta_e1 = 0;
    while(theta_e > SINGLEPI_CONSTANT) theta_e = theta_e-2*SINGLEPI_CONSTANT;
    while(theta_e <= -SINGLEPI_CONSTANT) theta_e = theta_e+2*SINGLEPI_CONSTANT;

    app_w_ = basicPDControl(pval,dval,theta_e,theta_e1,maxw);
    theta_e1 = theta_e ;

}// rotate to the target orientation by using PD control

void
Behaviour::accelerateLimit(const double &_acc_thresh, const bool &use_convected_acc)
{
    // ACC method
#define WHEELS 4
    const double WHEEL_DISTANCE=20.3;
    const double dcc_thresh = _acc_thresh*1.5;  //cm/s^2

    static float wheel_speed_old[WHEELS] = {0};
    float wheel_speed[WHEELS];
    float wheel_acc[WHEELS];
    float& Vx = app_vx_;
    float& Vy = app_vy_;
    float&  w = app_w_;

    if(WHEELS == 4)
    {
        wheel_speed[0]= ( 0.707*( Vx - Vy) - w*WHEEL_DISTANCE);
        wheel_speed[1]= ( 0.707*( Vx + Vy) - w*WHEEL_DISTANCE);
        wheel_speed[2]= ( 0.707*(-Vx + Vy) - w*WHEEL_DISTANCE);
        wheel_speed[3]= ( 0.707*(-Vx - Vy) - w*WHEEL_DISTANCE);
    }
    else
    {
        wheel_speed[0]= ( 0.866*Vx -  0.5*Vy - w*WHEEL_DISTANCE);
        wheel_speed[1]= (   0.0*Vx +      Vy - w*WHEEL_DISTANCE);
        wheel_speed[2]= ( -0.866*Vx - 0.5*Vy - w*WHEEL_DISTANCE);
    }
    float acc_thresh_ratio = 1;
    for(int i=0; i<WHEELS; i++)
    {
        wheel_acc[i] = wheel_speed[i]-wheel_speed_old[i];
        float acc_thresh_ratio_temp = 0;
        if( wheel_acc[i]*wheel_speed_old[i]>=0 ) //speed up
            acc_thresh_ratio_temp = fabs(wheel_acc[i])/_acc_thresh;
        else                                 //speed down
            acc_thresh_ratio_temp = fabs(wheel_acc[i])/dcc_thresh;
        if( acc_thresh_ratio_temp>acc_thresh_ratio )
            acc_thresh_ratio = acc_thresh_ratio_temp;
    }

    if( acc_thresh_ratio > 1 )
    {
        for(int i=0; i<WHEELS; i++)
        {
            wheel_acc[i] /= acc_thresh_ratio;
            wheel_speed[i] = wheel_speed_old[i] + wheel_acc[i];
        }
    }
    if(WHEELS==4)
    {
        w  = -(wheel_speed[0]+wheel_speed[1]+wheel_speed[2]+wheel_speed[3])/(4*WHEEL_DISTANCE);
        Vx =  (wheel_speed[0]+wheel_speed[1]-wheel_speed[2]-wheel_speed[3])/(2*1.414);
        Vy =  (wheel_speed[1]+wheel_speed[2]-wheel_speed[0]-wheel_speed[3])/(2*1.414);
    }
    else
    {
        Vx = ( 0.577*wheel_speed[0]  + 0 * wheel_speed[1] -  wheel_speed[2] * 0.577);
        Vy = (-0.333*wheel_speed[0]  + 0.667*wheel_speed[1] - wheel_speed[2]*0.333);
        w  = (-wheel_speed[0] - wheel_speed[1] - wheel_speed[2] )/(3*WHEEL_DISTANCE);
    }

    if(hypot(Vx,Vy)*fabs(w)*0.03>_acc_thresh)
    {
        float v_wheel=0;
        for(int i=0; i<WHEELS; i++)
        {
            if( fabs(wheel_speed[i])>v_wheel )
                v_wheel = fabs(wheel_speed[i]);
        }
        if(v_wheel<_acc_thresh) v_wheel = _acc_thresh;
        for(int i=0; i<WHEELS; i++)
        {
            wheel_speed[i] *= (1-_acc_thresh/v_wheel);
        }
        if(WHEELS==4)
        {
            w  = -(wheel_speed[0]+wheel_speed[1]+wheel_speed[2]+wheel_speed[3])/(4*WHEEL_DISTANCE);
            Vx =  (wheel_speed[0]+wheel_speed[1]-wheel_speed[2]-wheel_speed[3])/(2*1.414);
            Vy =  (wheel_speed[1]+wheel_speed[2]-wheel_speed[0]-wheel_speed[3])/(2*1.414);
        }
        else
        {
            Vx = ( 0.577*wheel_speed[0]  + 0 * wheel_speed[1] -  wheel_speed[2] * 0.577);
            Vy = (-0.333*wheel_speed[0]  + 0.667*wheel_speed[1] - wheel_speed[2]*0.333);
            w  = (-wheel_speed[0] - wheel_speed[1] - wheel_speed[2] )/(3*WHEEL_DISTANCE);
        }
    }
    if(use_convected_acc)
    {
        //        float Vx_r = worldmodelinfo_.robot_vel_.x_;
        //        float Vy_r = worldmodelinfo_.robot_vel_.y_;
        //        float  w_r = worldmodelinfo_.robot_omega_;
        float temp = Vx;
        Vx -=-Vy*w*0.1;
        Vy -= temp*w*0.1;
    }
    for(int i=0; i<WHEELS; i++)
    {
        wheel_speed_old[i] = wheel_speed[i];
    }
}
void
Behaviour::clearBehaviorState(){
    app_vx_ = 0;
    app_vy_ = 0;
    app_w_  = 0;
}

void
Behaviour::setAppvx(double vx){
    app_vx_ = vx;
}
void
Behaviour::setAppvy(double vy){
    app_vy_ = vy;
}
void
Behaviour::setAppw(double w){
    app_w_ = w;
}
