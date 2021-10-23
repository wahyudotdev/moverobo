#ifndef _kinematic_h
#define _kinematic_h

#include <Arduino.h>
#include <motor.h>

enum Base {
    BASE_DIFF_DRIVE,
    BASE_OMNI_Y,
    BASE_OMNI_X,
    BASE_MECHANUM
};

// Robot movement mode
enum Mode{
    DIRECT,
    PIVOT,
    HEADING,
};

class Kinematic
{
private:
    Base base;
    Motor *m1, *m3, *m2, *m4;
    float r_base = 0.16; // jari-jari base (khusus roda 3)
    double sqrt3 = 1.732050807568877193176604123436845839023590087890625;
    float d_wheel = 0.065;
    float last_goal_heading;

public:
    float radToDeg(float rad);
    float degToRad(float deg);
    volatile float pos_x, pos_y, pos_th;
    Kinematic(Base base);
    void setMotor(Motor &m1, Motor &m2);
    void setMotor(Motor &m1, Motor &m2, Motor &m3);
    void setMotor(Motor &m1, Motor &m2, Motor &m3, Motor &m4);
    void setSpeed(float linear_x, float linear_y, float linear_z, float angular_x, float angular_y, float angular_z);
    void setSpeed(float linear_x, float linear_y, float angular_z);
    void calculatePosition();
    float getGoalDistance(float goal_x, float goal_y);
    float getGoalHeading(float goal_x, float goal_y, bool normalize_pi);
    void setBaseRadius(float r_base);
};

#endif