#include <cmath>
#ifndef POINT_H
#define POINT_H
#include <project2/point.h>
#include <project2/traj.h>
#endif

class PID{
public:
    PID();

    //this function makes control output using arguments which are the current value and the target setpoint.
    float get_control(point car_pose, point goal_pose);
    void reset();
private:
    float error;
    float error_sum;
    float error_diff;
    float Kp;
    float Ki;
    float Kd;
};
