#include <project1/pid.h>

PID::PID(){

    /* TO DO
     *
     * find appropriate value of the coefficients for PID control, and initialize them.
     *
    */
    Kp = 1;
    Ki = 1;
    Kd = 1;
    error = 0;
    error_sum = 0;
    error_diff = 0;
}

float PID::get_control(point car_pose, point goal_pose){

    float ctrl;

    /* TO DO
     *
     * implement pid algorithm
     *
    */
    error_diff = goal_pose.th - car_pose.th - error;
    error = goal_pose.th - car_pose.th;
    error_sum = error_sum + error;

    ctrl = Kp * error + Ki * 0.1 * error_sum + (Kd/0.1) * error_diff;

    return ctrl;
}
