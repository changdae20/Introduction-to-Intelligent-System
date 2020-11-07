#include <project2/pid.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846

PID::PID(){

    /* TO DO
     *
     * find appropriate values for PID contorl, and initialize them.
     *
    */

    error = 0;
    error_sum = 0;
    error_diff = 0;
    Kp = 1.5;
    Ki = 0;
    Kd = 5; 
}

void PID::reset() {
    error = 0;
    error_sum = 0;
    error_diff = 0;
}

float PID::get_control(point car_pose, point goal_pose){

    /* TO DO
     *
     * implement pid algorithm
     *
    */
    
}
