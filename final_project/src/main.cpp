//state definition
#define INIT 0
#define PATH_PLANNING 1
#define RUNNING 2
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <pwd.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"

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

//parameters we should adjust : K, margin, MaxStep
int margin = 6;
int K = 10000;
double MaxStep = 0.5;
int waypoint_margin = 24;

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;

//FSM state
int state;

//function definition
void setcmdvel(double v, double w);
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void set_waypoints();
void generate_path_RRT();

int main(int argc, char** argv){
    ros::init(argc, argv, "slam_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/high_level/ackermann_cmd_mux/input/nav_0",1);
    
    ros::Subscriber gazebo_pose_sub = n.subscribe("/amcl_pose", 100, callback_state);

    printf("Initialize topics\n");
    
    // FSM
    state = INIT;
    bool running = true;
    ros::Rate control_rate(60);
    int look_ahead_idx;

    while(running){
        switch (state) {
        case INIT: {
            look_ahead_idx = 0;
            // Load Map
            char* user = getpwuid(getuid())->pw_name;
            cv::Mat map_org = cv::imread((std::string("/home/") + std::string(user) +
                              std::string("/catkin_ws/src/final_project/src/final.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.7;
            world_x_max = 4.7;
            world_y_min = -10.2;
            world_y_max = 10.2;
            res = 0.05;
            printf("Load map\n");

             if(! map.data )                              // Check for invalid input
            {
                printf("Could not open or find the image\n");
                return -1;
            }
            state = PATH_PLANNING;
        } break;

        case PATH_PLANNING:
            
            // Set Way Points
            set_waypoints();
            printf("Set way points\n");

            // RRT
            generate_path_RRT();
            printf("Generate RRT\n");

            ros::spinOnce();
            ros::Rate(0.33).sleep();

            printf("Initialize ROBOT\n");
            state = RUNNING;

        case RUNNING: {
            //TODO 3

            point goal;
            PID pid_ctrl;
            goal.x = path_RRT[look_ahead_idx].x;
            goal.y = path_RRT[look_ahead_idx].y;
            goal.th = path_RRT[look_ahead_idx].th;

            float ctrl_value = pid_ctrl.get_control(robot_pose, goal);
            float max_steering = 0.3;

            if (fabs(ctrl_value) > max_steering)
                ctrl_value = max_steering * ctrl_value / fabs(ctrl_value);

            //printf("(%.3f, %.3f) to (%.3f, %.3f)\n", robot_pose.x, robot_pose.y, goal.x, goal.y);

            setcmdvel(0.5, ctrl_value);
            cmd_vel_pub.publish(cmd);

            if (rrtTree::distance(path_RRT[look_ahead_idx], robot_pose) < (look_ahead_idx == path_RRT.size()-1 ? 0.2 : 0.5) 
                && look_ahead_idx < path_RRT.size()) {
                look_ahead_idx++;
                pid_ctrl.reset();
            }
            if(look_ahead_idx == path_RRT.size()) state = FINISH;

			ros::spinOnce();
			control_rate.sleep();

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

void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs){
    robot_pose.x = msgs->pose.pose.position.x;
    robot_pose.y = msgs->pose.pose.position.y;
    robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
    //printf("x,y : %f,%f \n",robot_pose.x,robot_pose.y);
}

void set_waypoints()
{
    point waypoint_candid[8];

    // Starting point. (Fixed)
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 8.5;


    //TODO 2
    // Set your own waypoints.
    // The car should turn around the outer track once, and come back to the starting point.
    // This is an example.
    waypoint_candid[1].x = 3.0;
    waypoint_candid[1].y = 8.0;
    waypoint_candid[2].x = 2.5;
    waypoint_candid[2].y = -8.5;
    waypoint_candid[3].x = -3.0;
    waypoint_candid[3].y = -7.0;
    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 8.5;
    // waypoint_candid[1].x = -3.5;
    // waypoint_candid[1].y = 8.5;
    // waypoint_candid[1].th = 1.0;
    waypoint_candid[4].th = 1.0;


    // Waypoints for arbitrary goal points.
    // TA will change this part before scoring.
    // This is an example.
    waypoint_candid[5].x = 1.5;
    waypoint_candid[5].y = 1.5;
    waypoint_candid[6].x = -2.0;
    waypoint_candid[6].y = -3.0;
    waypoint_candid[7].x = 1;
    waypoint_candid[7].y = -4.5;

    int order[] = {0,1,2,3,4,5,6,7};
    int order_size = 8;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void generate_path_RRT()
{   
    //TODO 1
    int size = waypoints.size();
    int failed = 0, max_failure = 10;
    time_t start_time = time(NULL);
	std::vector< std::vector<traj> > path_to_waypoint;

    // outer path setting
    {   // to fold the hard-coded code
        path_RRT.push_back(traj(-3.00, 8.45, -0.21, 0.325, 0.0));
        path_RRT.push_back(traj(-2.51, 8.40, 0.02, 0.325, 0.0));
        path_RRT.push_back(traj(-2.02, 8.48, 0.30, 0.325, 0.0));
        path_RRT.push_back(traj(-1.54, 8.61, 0.22, 0.325, 0.0));
        path_RRT.push_back(traj(-1.05, 8.70, 0.16, 0.325, 0.0));
        path_RRT.push_back(traj(-0.55, 8.72, -0.08, 0.325, 0.0));
        path_RRT.push_back(traj(-0.06, 8.74, 0.16, 0.325, 0.0));
        path_RRT.push_back(traj(0.43, 8.78, 0.00, 0.325, 0.0));
        path_RRT.push_back(traj(0.93, 8.72, -0.24, 0.325, 0.0));
        path_RRT.push_back(traj(1.38, 8.54, -0.54, 0.325, 0.0));
        path_RRT.push_back(traj(1.82, 8.30, -0.44, 0.325, 0.0));
        path_RRT.push_back(traj(2.20, 8.17, -0.22, 0.325, 0.0));
        path_RRT.push_back(traj(2.66, 8.01, -0.45, 0.325, 0.0));
        path_RRT.push_back(traj(2.96, 7.88, -0.41, 0.325, 0.0));
        path_RRT.push_back(traj(3.00, 8.00, 0.00, 0.325, 0.0));
        path_RRT.push_back(traj(3.42, 7.73, -0.72, 0.325, 0.0));
        path_RRT.push_back(traj(3.75, 7.36, -0.96, 0.325, 0.0));
        path_RRT.push_back(traj(3.97, 6.92, -1.26, 0.325, 0.0));
        path_RRT.push_back(traj(4.06, 6.43, -1.50, 0.325, 0.0));
        path_RRT.push_back(traj(4.05, 5.94, -1.67, 0.325, 0.0));
        path_RRT.push_back(traj(4.01, 5.44, -1.64, 0.325, 0.0));
        path_RRT.push_back(traj(3.98, 4.95, -1.64, 0.325, 0.0));
        path_RRT.push_back(traj(3.88, 4.46, -1.88, 0.325, 0.0));
        path_RRT.push_back(traj(3.73, 3.99, -1.91, 0.325, 0.0));
        path_RRT.push_back(traj(3.61, 3.51, -1.71, 0.325, 0.0));
        path_RRT.push_back(traj(3.59, 3.01, -1.51, 0.325, 0.0));
        path_RRT.push_back(traj(3.63, 2.51, -1.45, 0.325, 0.0));
        path_RRT.push_back(traj(3.68, 2.02, -1.50, 0.325, 0.0));
        path_RRT.push_back(traj(3.70, 1.52, -1.55, 0.325, 0.0));
        path_RRT.push_back(traj(3.71, 1.02, -1.56, 0.325, 0.0));
        path_RRT.push_back(traj(3.73, 0.53, -1.49, 0.325, 0.0));
        path_RRT.push_back(traj(3.81, 0.04, -1.33, 0.325, 0.0));
        path_RRT.push_back(traj(3.95, -0.44, -1.27, 0.325, 0.0));
        path_RRT.push_back(traj(4.05, -0.93, -1.44, 0.325, 0.0));
        path_RRT.push_back(traj(4.07, -1.42, -1.63, 0.325, 0.0));
        path_RRT.push_back(traj(4.08, -1.92, -1.47, 0.325, 0.0));
        path_RRT.push_back(traj(4.09, -2.42, -1.64, 0.325, 0.0));
        path_RRT.push_back(traj(4.07, -2.92, -1.58, 0.325, 0.0));
        path_RRT.push_back(traj(4.09, -3.41, -1.50, 0.325, 0.0));
        path_RRT.push_back(traj(4.08, -3.90, -1.65, 0.325, 0.0));
        path_RRT.push_back(traj(4.03, -4.40, -1.73, 0.325, 0.0));
        path_RRT.push_back(traj(3.94, -4.89, -1.74, 0.325, 0.0));
        path_RRT.push_back(traj(3.84, -5.38, -1.83, 0.325, 0.0));
        path_RRT.push_back(traj(3.73, -5.86, -1.76, 0.325, 0.0));
        path_RRT.push_back(traj(3.61, -6.34, -1.86, 0.325, 0.0));
        path_RRT.push_back(traj(3.44, -6.81, -2.00, 0.325, 0.0));
        path_RRT.push_back(traj(3.21, -7.25, -2.10, 0.325, 0.0));
        path_RRT.push_back(traj(3.05, -7.60, -1.88, 0.325, 0.0));
        path_RRT.push_back(traj(2.93, -7.90, -2.02, 0.325, 0.0));
        path_RRT.push_back(traj(2.77, -8.20, -2.13, 0.325, 0.0));
        path_RRT.push_back(traj(2.51, -8.52, -2.38, 0.325, 0.0));
        path_RRT.push_back(traj(2.50, -8.50, 0.00, 0.325, 0.0));
        path_RRT.push_back(traj(2.10, -8.78, -2.67, 0.325, 0.0));
        path_RRT.push_back(traj(1.64, -8.97, -2.85, 0.325, 0.0));
        path_RRT.push_back(traj(1.16, -9.05, -3.11, 0.325, 0.0));
        path_RRT.push_back(traj(0.66, -9.02, 3.01, 0.325, 0.0));
        path_RRT.push_back(traj(0.18, -8.89, 2.71, 0.325, 0.0));
        path_RRT.push_back(traj(-0.24, -8.62, 2.43, 0.325, 0.0));
        path_RRT.push_back(traj(-0.64, -8.33, 2.63, 0.325, 0.0));
        path_RRT.push_back(traj(-1.10, -8.14, 2.86, 0.325, 0.0));
        path_RRT.push_back(traj(-1.57, -7.97, 2.77, 0.325, 0.0));
        path_RRT.push_back(traj(-2.01, -7.74, 2.52, 0.325, 0.0));
        path_RRT.push_back(traj(-2.43, -7.49, 2.70, 0.325, 0.0));
        path_RRT.push_back(traj(-2.86, -7.22, 2.47, 0.325, 0.0));
        path_RRT.push_back(traj(-3.09, -7.00, 2.32, 0.325, 0.0));
        path_RRT.push_back(traj(-3.00, -7.00, 0.00, 0.325, 0.0));
        path_RRT.push_back(traj(-3.32, -6.61, 2.19, 0.325, 0.0));
        path_RRT.push_back(traj(-3.51, -6.35, 2.24, 0.325, 0.0));
        path_RRT.push_back(traj(-3.70, -6.06, 2.04, 0.325, 0.0));
        path_RRT.push_back(traj(-3.87, -5.60, 1.82, 0.325, 0.0));
        path_RRT.push_back(traj(-3.92, -5.11, 1.54, 0.325, 0.0));
        path_RRT.push_back(traj(-3.84, -4.62, 1.25, 0.325, 0.0));
        path_RRT.push_back(traj(-3.72, -4.14, 1.42, 0.325, 0.0));
        path_RRT.push_back(traj(-3.69, -3.64, 1.61, 0.325, 0.0));
        path_RRT.push_back(traj(-3.69, -3.15, 1.53, 0.325, 0.0));
        path_RRT.push_back(traj(-3.64, -2.65, 1.40, 0.325, 0.0));
        path_RRT.push_back(traj(-3.63, -2.16, 1.69, 0.325, 0.0));
        path_RRT.push_back(traj(-3.71, -1.66, 1.78, 0.325, 0.0));
        path_RRT.push_back(traj(-3.76, -1.17, 1.57, 0.325, 0.0));
        path_RRT.push_back(traj(-3.75, -0.67, 1.54, 0.325, 0.0));
        path_RRT.push_back(traj(-3.70, -0.19, 1.42, 0.325, 0.0));
        path_RRT.push_back(traj(-3.70, 0.30, 1.70, 0.325, 0.0));
        path_RRT.push_back(traj(-3.74, 0.80, 1.61, 0.325, 0.0));
        path_RRT.push_back(traj(-3.71, 1.30, 1.42, 0.325, 0.0));
        path_RRT.push_back(traj(-3.68, 1.79, 1.59, 0.325, 0.0));
        path_RRT.push_back(traj(-3.75, 2.28, 1.84, 0.325, 0.0));
        path_RRT.push_back(traj(-3.84, 2.76, 1.68, 0.325, 0.0));
        path_RRT.push_back(traj(-3.84, 3.26, 1.46, 0.325, 0.0));
        path_RRT.push_back(traj(-3.82, 3.75, 1.61, 0.325, 0.0));
        path_RRT.push_back(traj(-3.83, 4.25, 1.56, 0.325, 0.0));
        path_RRT.push_back(traj(-3.83, 4.74, 1.59, 0.325, 0.0));
        path_RRT.push_back(traj(-3.82, 5.24, 1.52, 0.325, 0.0));
        path_RRT.push_back(traj(-3.74, 5.73, 1.27, 0.325, 0.0));
        path_RRT.push_back(traj(-3.56, 6.19, 1.12, 0.325, 0.0));
        path_RRT.push_back(traj(-3.50, 6.66, 1.33, 0.325, 0.0));
        path_RRT.push_back(traj(-3.61, 7.15, 1.59, 0.325, 0.0));
        path_RRT.push_back(traj(-3.61, 7.48, 1.61, 0.325, 0.0));
        path_RRT.push_back(traj(-3.65, 7.80, 1.61, 0.325, 0.0));
        path_RRT.push_back(traj(-3.64, 8.18, 1.83, 0.325, 0.0));
        path_RRT.push_back(traj(-3.52, 8.48, 2.02, 0.325, 0.0));
        path_RRT.push_back(traj(-3.50, 8.50, 1.00, 0.325, 0.0));
    }

	for (int i = 4; i < size - 1; i++) {
		rrtTree Tree = rrtTree(waypoints[i], waypoints[i + 1], map, map_origin_x, map_origin_y, res, margin);
		Tree.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);

        // Tree.visualizeTree(); Tree.visualizeTree(); getchar();

		std::vector<traj> temp_path = Tree.backtracking_traj();

		bool well_made = false;
        if (!temp_path.empty())
            well_made = rrtTree::distance(temp_path.front(), waypoints[i + 1]) < 0.5;
        else
            well_made = rrtTree::distance(waypoints[i], waypoints[i + 1]) < 0.5;

        std::vector<traj> start_waypoint;
		traj waypoint;
		waypoint.x = waypoints[i + 1].x;
		waypoint.y = waypoints[i + 1].y;
		waypoint.th = waypoints[i + 1].th;
		waypoint.d = 0.325;
		waypoint.alpha = 0;
		start_waypoint.push_back(waypoint);
		start_waypoint.insert(start_waypoint.end(), temp_path.begin(), temp_path.end());
        // Tree.visualizeTree(start_waypoint); Tree.visualizeTree(start_waypoint); getchar();
        if (well_made) {
			if (!temp_path.empty()) waypoints[i + 1].th = start_waypoint[1].th;
            else waypoints[i + 1].th = waypoints[i].th;
			path_to_waypoint.push_back(start_waypoint);
            printf("generate path %d to %d\n", i, i+1);
            failed = 0;
		} else {
            if (failed + 1 >= max_failure) {
                printf("Too much failure to plan path, it'll give you the best result only until waypoint %d\n", i);
                break;
            }
            if (i <= 4) {
                printf("cancel path %d to %d\n", i, i+1);
                i = i - 1;
            } else {
                printf("delete path %d to %d\n", i-1, i);
                i = i - 2;
                path_to_waypoint.pop_back();
            }
            printf("failed count: %d / %d\n", ++failed, max_failure);
        }

        if (time(NULL) - start_time > 210) {
            printf("Too much time to generate the path\n");
            break;
        }
    }

    for (int i = 0; i < path_to_waypoint.size(); i++) {
		while (!path_to_waypoint[i].empty()) {
			path_RRT.push_back(path_to_waypoint[i].back());
			path_to_waypoint[i].pop_back();
		}
	}

    // For Debugging
    //for(int i=0; i<path_RRT.size();i++){
    //    printf("path_RRT.push_back(traj(%.2f, %.2f, %.2f, 0.325, 0.0));\n",path_RRT[i].x,path_RRT[i].y,path_RRT[i].th);
    //}
}
