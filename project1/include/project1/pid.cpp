#include <project1/pid.h>

PID::PID(){

    /* TO DO
     *
     * find appropriate value of the coefficients for PID control, and initialize them.
     *
    */
    
    float ctrl_rate = 10;
    error = 0;
    error_sum = 0;
    error_diff = 0;
    Kp = 1;
    Ki = 0.001;
    Kd = 0.001;

    Kd *= ctrl_rate;
    Ki /= ctrl_rate;
    Kp += Ki + Kd;
}

float PID::get_control(point car_pose, point goal_pose){

    float ctrl;

    /* TO DO
     *
     * implement pid algorithm
     *
    */

    error = atan2(goal_pose.y-car_pose.y, goal_pose.x-car_pose.x) - car_pose.th;
    ctrl = Kp * error + Ki * error_sum - Kd * error_diff;
    error_sum += error;
    error_diff = error;

    return ctrl;
}
