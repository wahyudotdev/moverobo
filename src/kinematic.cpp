#include "kinematic.h"

Kinematic::Kinematic(Base base)
{
    this->base = base;
}
void Kinematic::setMotor(Motor &m1, Motor &m2, Motor &m3)
{
    this->m1 = &m1;
    this->m2 = &m2;
    this->m3 = &m3;
}

void Kinematic::setMotor(Motor &m1, Motor &m2)
{
    this->m1 = &m1;
    this->m2 = &m2;
}

void Kinematic::setSpeed(float linear_x, float linear_y, float linear_z, float angular_x, float angular_y, float angular_z)
{
    if (base == BASE_OMNI_Y)
    {
        float inv_m1 = (0.58 * linear_x) + (-0.33 * linear_y) + (0.33 * angular_z);
        float inv_m2 = (0 * linear_x) + (0.67 * linear_y) + (0.33 * angular_z);
        float inv_m3 = (-0.58 * linear_x) + (-0.33 * linear_y) + (0.33 * angular_z);
        float sp_m1 = (inv_m1 / (PI * d_wheel)) * 60;
        float sp_m2 = (inv_m2 / (PI * d_wheel)) * 60;
        float sp_m3 = (inv_m3 / (PI * d_wheel)) * 60;
        m1->speed(sp_m1);
        m2->speed(sp_m2);
        m3->speed(sp_m3);
    }
    if (base == BASE_DIFF_DRIVE)
    {
    }
}

void Kinematic::setSpeed(float lin_x, float lin_y, float ang_z)
{
    if (base == BASE_DIFF_DRIVE)
    {
        float inv_m1 = lin_y - (0.4 * ang_z);
        float inv_m2 = -1 * (lin_y + (0.4 * ang_z));
        float sp_m1 = (inv_m1 / (PI * d_wheel)) * 60;
        float sp_m2 = (inv_m2 / (PI * d_wheel)) * 60;
        m1->speed(sp_m1);
        m2->speed(sp_m2);
        // Serial.printf("m1 : %.2f m2 : %.2f\n", sp_m1, sp_m2);
    }

    if (base == BASE_OMNI_Y)
    {
        float inv_m1 = (-0.33 * lin_x) + (0.58 * lin_y) + (0.33 * ang_z);
        float inv_m2 = (0.67 * lin_x) + (0 * lin_y) + (0.33 * ang_z);
        float inv_m3 = (-0.33 * lin_x) + (-0.58 * lin_y) + (0.33 * ang_z);
        float sp_m1 = (inv_m1 / (PI * d_wheel)) * 60;
        float sp_m2 = (inv_m2 / (PI * d_wheel)) * 60;
        float sp_m3 = (inv_m3 / (PI * d_wheel)) * 60;
        m1->speed(sp_m1);
        m2->speed(sp_m2);
        m3->speed(sp_m3);
    }
}

void Kinematic::calculatePosition()
{
    if (base == BASE_DIFF_DRIVE)
    {

        float heading = pos_th;
        float leftTravel = m1->speed_ms;
        float rightTravel = m2->speed_ms;
        float deltaTravel = (rightTravel + leftTravel) / 2; // Menghitung kecepatan maju robot (m/s)
        float deltaTheta = (rightTravel - leftTravel) / (this->r_base * 2); // Menghitung kecepatan putar robot (rad/s)
        float deltaX = deltaTravel * cos(heading); // Menghitung jarak perpindahan X robot sebenarnya di lapangan berdasarkan sudut hadap robot
        float deltaY = deltaTravel * sin(heading); // Menghitung jarak perpindahan X robot sebenarnya di lapangan berdasarkan sudut hadap robot
        pos_th += deltaTheta;
        pos_x += deltaX;
        pos_y += deltaY;
    }

    if (base == BASE_OMNI_Y)
    {
        float v1 = (m1->speed_ms);
        float v2 = (m2->speed_ms);
        float v3 = (m3->speed_ms);
        float vmx = (2 * v2 - v1 - v3) / 3;
        float vmy = ((sqrt3 * v3) - (sqrt3 * v1)) / 3;
        float heading = (v1 + v2 + v3) / (3 * (r_base * 2 * 3));
        pos_th += heading * PI / 180;
        pos_x += (cos(pos_th) * vmx) - (sin(pos_th) * vmy);
        pos_y += (sin(pos_th) * vmx) + (cos(pos_th) * vmy);
    }
}

float Kinematic::getGoalDistance(float goal_x, float goal_y)
{
    float diff_x = goal_x - pos_x;
    float diff_y = goal_y - pos_y;
    float distance = sqrt((diff_y * diff_y) + (diff_x * diff_x));
    return distance;
}

float Kinematic::getGoalHeading(float goal_x, float goal_y, bool normalize_pi)
{
    float x_trans, y_trans, goal_heading, rotation;
    float diff_x = goal_x - pos_x;
    float diff_y = goal_y - pos_y;
    if (normalize_pi)
    {
        if (diff_x >= 0 and diff_y >= 0)
        {
            rotation = degToRad(0);
        }
        if (diff_x < 0 and diff_y >= 0)
        {
            rotation = degToRad(-90);
        }
        if (diff_x < 0 and diff_y < 0)
        {
            rotation = degToRad(-180);
        }
        if (diff_x >= 0 and diff_y < 0)
        {
            rotation = degToRad(-270);
        }
    }
    else{
        rotation = degToRad(0);
    }

    x_trans = cos(rotation) * diff_x + -sin(rotation) * diff_y;
    y_trans = sin(rotation) * diff_x + cos(rotation) * diff_y;

    goal_heading = atan2(y_trans, x_trans);
    // Serial.printf("diff_x : %.2f diff_y %.2f :  x_trans : %.2f y_trans: %.2f heading: %.2f\n", diff_x, diff_y, x_trans, y_trans, radToDeg(goal_heading));
    return goal_heading;
}

float Kinematic::radToDeg(float rad)
{
    return rad * 180 / PI;
}

float Kinematic::degToRad(float deg)
{
    return deg * PI / 180;
}

void Kinematic::setBaseRadius(float r_base){
    this->r_base = r_base;
}