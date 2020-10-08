#include <project1/pid.h>
#include <math.h>

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

    error_diff = atan2(goal_pose.y-car_pose.y,goal_pose.x-car_pose.x) - car_pose.th - error;
    error = atan2(goal_pose.y-car_pose.y,goal_pose.x-car_pose.x) - car_pose.th;
    error_sum = error_sum + error;

    ctrl = Kp * error + Ki * 0.1 * error_sum + (Kd/0.1) * error_diff;

    return ctrl;
}
