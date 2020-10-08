#include <ros/ros.h>
#include <project1/pid.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <cmath>

//car pose variable
point car_pose;

//vector type variable, which contains way points. the racecar has to fallowing these way points.
//"point" is data type defined by TA. and it contains member variable x,y,th which are x,y coordinate and heading from x axis of car, respectively.
std::vector<point> path;

void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void setpath();

int main(int argc, char** argv){
    ros::init(argc,argv,"pidmain");
    ros::NodeHandle n;

    ros::Subscriber gazebo_pose_sub = n.subscribe("/gazebo/model_states",1,callback_state);
    ros::ServiceClient gazebo_spawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    ros::Publisher car_ctrl_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",1);

    //initalize variable "path".
    setpath();

    /* Do not revise or delete this code */
    //visualize path to GAZEBO.
    for(int i = 0; i < path.size(); i++){
        printf("[%d] way point was spawned\n",i);
        gazebo_msgs::SpawnModel model;
        model.request.model_xml = std::string("<robot name=\"simple_ball\">") +
                std::string("<link name=\"ball\">") +
                std::string("<inertial>") +
                std::string("<mass value=\"1.0\" />") +
                std::string("<origin xyz=\"0 0 0\" />") +
                std::string("<inertia  ixx=\"1.0\" ixy=\"0.0\"  ixz=\"0.0\"  iyy=\"1.0\"  iyz=\"0.0\"  izz=\"1.0\" />") +
                std::string("</inertial>") +
                std::string("<visual>") +
                std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                std::string("<geometry>") +
                std::string("<sphere radius=\"0.09\"/>") +
                std::string("</geometry>") +
                std::string("</visual>") +
                std::string("<collision>") +
                std::string("<origin xyz=\"0 0 0\" rpy=\"0 0 0\" />") +
                std::string("<geometry>") +
                std::string("<sphere radius=\"0\"/>") +
                std::string("</geometry>") +
                std::string("</collision>") +
                std::string("</link>") +
                std::string("<gazebo reference=\"ball\">") +
                std::string("<mu1>10</mu1>") +
                std::string("<mu2>10</mu2>") +
                std::string("<material>Gazebo/Blue</material>") +
                std::string("<turnGravityOff>true</turnGravityOff>") +
                std::string("</gazebo>") +
                std::string("</robot>");

        std::ostringstream ball_name;
        ball_name << i;
        model.request.model_name = ball_name.str();
        model.request.reference_frame = "world";
        model.request.initial_pose.position.x = path[i].x;
        model.request.initial_pose.position.y = path[i].y;
        model.request.initial_pose.position.z = 2.0;
        model.request.initial_pose.orientation.w = 0.0;
        model.request.initial_pose.orientation.x = 0.0;
        model.request.initial_pose.orientation.y = 0.0;
        model.request.initial_pose.orientation.z = 0.0;

        gazebo_spawn.call(model);

        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    // set the car on the initial position
    geometry_msgs::Pose model_pose;
    model_pose.position.x = path[0].x;
    model_pose.position.y = path[0].y;
    model_pose.position.z = 0.3;
    model_pose.orientation.x = 0.0;
    model_pose.orientation.y = 0.0;
    model_pose.orientation.z = 0.0;
    model_pose.orientation.w = 0.0;

    geometry_msgs::Twist model_twist;
    model_twist.linear.x = 0.0;
    model_twist.linear.y = 0.0;
    model_twist.linear.z = 0.0;
    model_twist.angular.x = 0.0;
    model_twist.angular.y = 0.0;
    model_twist.angular.z = 0.0;

    gazebo_msgs::ModelState modelstate;
    modelstate.model_name = "racecar";
    modelstate.reference_frame = "world";
    modelstate.pose = model_pose;
    modelstate.twist = model_twist;

    gazebo_msgs::SetModelState setmodelstate;
    setmodelstate.request.model_state = modelstate;

    gazebo_set.call(setmodelstate);
    ros::spinOnce();
    ros::Rate(0.5).sleep();
    // finish initialization

    /* controller */
    
    int current_goal = 1;
    PID pid_ctrl;
    ackermann_msgs::AckermannDriveStamped drive_msg_stamped;

    // control rate, 10 Hz
    ros::Rate control_rate(10);
    while(ros::ok()){
		
        /*TO DO
         * 1. make control value for steering angle using PID class. An instance is predefined as "pid_ctrl".
         * 2. publish control to racecar.
         *    use predefined publisher, "car_ctrl_pub" and use predefined variable, "drive_msg_stamped".
         * 3. check whether pioneer reached a currently following way point or not.
         *    calculate distance between current pose of robot and currently following way point.
         *    if distance is less than 0.2m (you can change this threshold), pursue next way point.
         * 4. check whether car reached final way point(end of path). if it is, terminate controller.
        */
        
        ros::spinOnce();
        control_rate.sleep();
        printf("car pose : %.2f,%.2f,%.2f \n", car_pose.x, car_pose.y, car_pose.th);
    }

    return 0;
}

// get position data from ros msgs
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs){
    for(int i; i < msgs->name.size(); i++){
        if(std::strcmp(msgs->name[i].c_str(),"racecar") == 0){
            car_pose.x = msgs->pose[i].position.x;
            car_pose.y = msgs->pose[i].position.y;
            car_pose.th = tf::getYaw(msgs->pose[i].orientation);
        }
    }
}

void setpath(){

    point point0;    point0.x = -4.0;    point0.y = -4.0;    point0.th = 0.0;
    point point1;    point1.x = -4.0;    point1.y = 0.0;    point1.th = 0.5 * M_PI;
    point point2;    point2.x = -4.0;    point2.y = 4.0;    point2.th = 0.25 * M_PI;

    point point3;    point3.x = 0.0;    point3.y = 4.0;    point3.th = -0.5 * M_PI;
    point point4;    point4.x = 0.0;    point4.y = 0.0;    point4.th = -0.5 * M_PI;
    point point5;    point5.x = 0.0;    point5.y = -4.0;    point5.th = -0.25 * M_PI;

    point point6;    point6.x = 4.0;    point6.y = -4.0;    point6.th = 0.5 * M_PI;
    point point7;    point7.x = 4.0;    point7.y = 0.0;    point7.th = 0.5 * M_PI;
    point point8;    point8.x = 4.0;    point8.y = 4.0;    point8.th = 0.5 * M_PI;

    path.push_back(point0);    path.push_back(point1);    path.push_back(point2);
    path.push_back(point3);    path.push_back(point4);    path.push_back(point5);
    path.push_back(point6);    path.push_back(point7);    path.push_back(point8);
}
