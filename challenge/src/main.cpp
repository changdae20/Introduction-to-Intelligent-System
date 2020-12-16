//state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <pwd.h>
#include <project2/pid.h>
#include <math.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Path.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include "race/drive_param.h"
#include <iostream>
#include <fstream>
#include <string>

//map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

//TODO 1
//parameters students should adjust : K, margin, MaxStep
int margin = 7;
int K = 1000;
double MaxStep = 2.0;
/////////

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

//control
nav_msgs::Path visual_path;

//robot
point robot_pose;
race::drive_param cmd;

//FSM state
int state;

//function definition
void set_waypoints();
void generate_path_RRT();
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void setcmdvel(double v, double w);

int main(int argc, char** argv){
    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Publisher cmd_vel_pub = n.advertise<race::drive_param>("/drive_parameters",100); 
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("/rrt_path",100);
    
    ros::Subscriber robot_pose_sub = n.subscribe("/amcl_pose", 100, callback_state);
    printf("Initialize topics\n");

    // Load Map
/////////////////////////////////////////////// Temproary map for hardward setup. it will be modified on the actual challenge day
    char* user = getpwuid(getuid())->pw_name;
    map = cv::imread((std::string("/home/nvidia/") +
                      std::string("catkin_ws/src/challenge/src/final.pgm")).c_str(),CV_LOAD_IMAGE_GRAYSCALE);
//////////////////////////////////////////////////////////////////////////////////////

    cv::transpose(map,map);
    cv::flip(map,map,1);
    map_y_range = map.cols;
    map_x_range = map.rows;
    map_origin_x = map_x_range/2.0 - 0.5;
    map_origin_y = map_y_range/2.0 - 0.5;

/////////////////////////////////////////////// Also, these numbers will be modified on the actual challenge day
    world_x_min = -1.7;
    world_x_max = 2.2;
    world_y_min = -2.6;
    world_y_max = 3.3;
    res = 0.05;
//////////////////////////////////////////////////

    printf("Load map\n");


    if(! map.data )                              // Check for invalid input
    {
        printf("Could not open or find the image\n");
        return -1;
    }

    // Set Way Points
    set_waypoints();
    printf("Set way points\n");

    //initial_pose
    robot_pose.x= waypoints[0].x;
    robot_pose.y= waypoints[0].y;
    robot_pose.th= waypoints[0].th;


    // RRT
    generate_path_RRT();
    printf("Generate RRT\n");

    // FSM
    state = INIT;
    bool running = true;
    ros::Rate control_rate(60);

    while(running){
        switch (state) {
        case INIT: {
        printf("path size : %d\n", path_RRT.size());
            state = RUNNING;
        } break;

        case RUNNING: {
			/////don't remove this///////
		    visual_path.poses.resize(path_RRT.size());
            for(int i =0; i < path_RRT.size(); i++){
                visual_path.header.frame_id = "map";
		        visual_path.header.stamp = ros::Time::now();
		        visual_path.poses[i].pose.position.x = path_RRT[i].x;			
		        visual_path.poses[i].pose.position.y = path_RRT[i].y;
            }
			////////////////////////////
			while(ros::ok()) {
            	path_pub.publish(visual_path);
				/* TODO 2
				****Copy your code in final project main.cpp
				****Usage of cmd_vel_pub is slightly different in real rccars
				******Before: cmd.drive.speed = 0.5; cmd.drive.steering_angle = 0.4;
				******After : cmd.velociy = 0.5; cmd.angle = 0.4;
				******(but you can use setcmdvel() function same as before)
				****Also, the scales of the control inputs are different in gazebo and real rccar. 
				****You have to calibrate those numbers during hardware setup week.
				
				****please check the below instructions, and tune your control inputs roughly before hardware setup week
				****
				****cmd.velocity (the numbers does not mean **m/s!!)
                ****-> Each rc car have its own minimum velocity values (0.3 or 0.45 or 0.5). Take this into account and prepare for the hardware setup.
				****cmd.angle
				****-> max , min angle value are 1 (35 degree) and -1(-35 degree). You may need to normalize your PID ctrl value
                
			*/
                point goal = rrtTree::traj2point(path_RRT[look_ahead_idx]);
                PID pid_ctrl;
                float velocity = 1.4;
                float ctrl_value = pid_ctrl.get_control(robot_pose, goal)*0.4/velocity;
                float max_steering = 0.3;

                if (fabs(ctrl_value) > max_steering)
                    ctrl_value = max_steering * ctrl_value / fabs(ctrl_value);

                // printf("%d: (%.3f, %.3f) to (%.3f, %.3f)\n", look_ahead_idx, robot_pose.x, robot_pose.y, goal.x, goal.y);

                setcmdvel(velocity, ctrl_value/0.61);
                cmd_vel_pub.publish(cmd);

                if (rrtTree::distance(path_RRT[look_ahead_idx], robot_pose) < (look_ahead_idx == path_RRT.size()-1 ? 0.3 : 0.5) 
                    && look_ahead_idx < path_RRT.size()) {
                    look_ahead_idx++;
                    pid_ctrl.reset();
                }
                if(look_ahead_idx == path_RRT.size()) state = FINISH;

                ros::spinOnce();
                control_rate.sleep();
			}
        } break;

        case FINISH: {
            setcmdvel(0,0);
            cmd_vel_pub.publish(cmd);
            running = false;
            ros::spinOnce();
            control_rate.sleep();
        } break;

        default: {
        } break;
        }
    }
    return 0;
}

void generate_path_RRT()
{
	/*TODO 3
	****copy your code in final project main.cpp
    ****delete tree->visualizeTree() and tree->visualizeTree(vec) */
	int size = waypoints.size();
    time_t start_time = time(NULL);
	std::vector< std::vector<traj> > path_to_waypoint;
    std::vector<point> last_points = waypoints;

	for (int i = 0; i < size - 1; i++) {
		rrtTree Tree = rrtTree(
            last_points[i], waypoints[i + 1],
            map, map_origin_x, map_origin_y, res, margin,
            world_x_max, world_x_min, world_y_max, world_y_min
        );
        if (i < OUTER_POINTS-1) {
            double x_max = std::max(3*waypoints[i+1].x-2*waypoints[i].x, std::max(waypoints[i].x, waypoints[i+1].x)+2);
            double x_min = std::min(3*waypoints[i+1].x-2*waypoints[i].x, std::min(waypoints[i].x, waypoints[i+1].x)-2);
            double y_max = std::max(3*waypoints[i+1].y-2*waypoints[i].y, std::max(waypoints[i].y, waypoints[i+1].y)+2);
            double y_min = std::min(3*waypoints[i+1].y-2*waypoints[i].y, std::min(waypoints[i].y, waypoints[i+1].y)-2);
            int k = (int)((x_max - x_min + 2) * (y_max - y_min + 2));
            Tree.generateRRT(x_max, x_min, y_max, y_min, k * 3, MaxStep/2);
        } else Tree.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);

        //Tree.visualizeTree(); Tree.visualizeTree(); getchar();

		std::vector<traj> start_waypoint = Tree.backtracking_traj();
		start_waypoint.push_back(rrtTree::point2traj(last_points[i]));
		bool well_made = rrtTree::distance(start_waypoint.front(), waypoints[i + 1]) < 0.5;

        if (well_made) {
			last_points[i + 1] = rrtTree::traj2point(start_waypoint.front());
			path_to_waypoint.push_back(start_waypoint);
            printf("generate path %d to %d\n\n", i, i+1);
            // Tree.visualizeTree(); Tree.visualizeTree(); getchar();
		} else {
            printf("failed to go to waypoint %d\n", i+1);
            // Tree.visualizeTree(); Tree.visualizeTree(); getchar();
            if (i <= 0) {
                printf("cancel path %d to %d\n\n", i, i+1);
                i = i - 1;
            } else {
                printf("delete path %d to %d\n\n", i-1, i);
                i = i - 2;
                path_to_waypoint.pop_back();
            }
        }
        if (time(NULL) - start_time > 270) {
            printf("Too much time to generate the path\n");
            break;
        }
    }

    double d_threshold = 0.6;
    path_RRT.push_back(rrtTree::point2traj(waypoints[0]));
    for (int i = 0; i < path_to_waypoint.size(); ++i) {
        while (!path_to_waypoint[i].empty()) {
            traj origin = path_RRT.back();
            traj goal = path_to_waypoint[i].back();
            int cut_cnt = 1;
            while (path_to_waypoint[i].back().d >= 1.5 * d_threshold) {
                path_RRT.push_back(
                    rrtTree::predict_point(
                        origin, goal, cut_cnt * d_threshold
                    )
                );
                ++cut_cnt;
                path_to_waypoint[i].back().d -= d_threshold;
            }
            path_RRT.push_back(path_to_waypoint[i].back());
            path_to_waypoint[i].pop_back();
        }
    }
}

void set_waypoints()
{
    point waypoint_candid[8];
    
    waypoint_candid[0].x = -1.05;
    waypoint_candid[0].y = 2.7;
    waypoint_candid[0].th = -3.141592/4;
    waypoint_candid[1].x = 0.1;
    waypoint_candid[1].y = -0.93;
    waypoint_candid[2].x = 1.7;
    waypoint_candid[2].y = -1.95;
 
    int order[] = {0,1,2};
    int order_size = 2;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs){
    robot_pose.x = msgs->pose.pose.position.x;
    robot_pose.y = msgs->pose.pose.position.y;
    robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
}

void setcmdvel(double vel, double deg){
    cmd.velocity = vel;
    cmd.angle = deg;
}
