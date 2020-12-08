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
int K = 3000;
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

            // printf("(%.3f, %.3f) to (%.3f, %.3f)\n", robot_pose.x, robot_pose.y, goal.x, goal.y);

            setcmdvel(1.0, ctrl_value);
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
    point waypoint_candid[7];

    // Starting point. (Fixed)
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 8.5;


    //TODO 2
    // Set your own waypoints.
    // The car should turn around the outer track once, and come back to the starting point.
    // This is an example.
    waypoint_candid[1].x = 2.2;
    waypoint_candid[1].y = 8.5;
    waypoint_candid[2].x = 2.5;
    waypoint_candid[2].y = -8.5;
    waypoint_candid[3].x = -2.5;
    waypoint_candid[3].y = -8.0;
    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 8.5;


    // Waypoints for arbitrary goal points.
    // TA will change this part before scoring.
    // This is an example.
    waypoint_candid[5].x = 1.5;
    waypoint_candid[5].y = 1.5;
    waypoint_candid[6].x = -2;
    waypoint_candid[6].y = -9.0;

    int order[] = {0,1,2,3,4,5,6};
    int order_size = 7;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void generate_path_RRT()
{   
    //TODO 1
    int size = waypoints.size();
	std::vector< std::vector<traj> > path_to_waypoint;
	for (int i = 0; i < size - 1; i++) {
		rrtTree Tree = rrtTree(waypoints[i], waypoints[i + 1], map, map_origin_x, map_origin_y, res, margin);
		Tree.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);

		std::vector<traj> temp_path = Tree.backtracking_traj();

		bool well_made = false;
        if (!temp_path.empty())
            well_made = rrtTree::distance(temp_path.front(), waypoints[i + 1]) < 0.5;

        std::vector<traj> start_waypoint;
		traj waypoint;
		waypoint.x = waypoints[i + 1].x;
		waypoint.y = waypoints[i + 1].y;
		waypoint.th = waypoints[i + 1].th;
		waypoint.d = 0.325;
		waypoint.alpha = 0;
		start_waypoint.push_back(waypoint);
		start_waypoint.insert(start_waypoint.end(), temp_path.begin(), temp_path.end());

        if (well_made) {
			if (!temp_path.empty())
				waypoints[i + 1].th = start_waypoint[1].th;
			path_to_waypoint.push_back(start_waypoint);
            printf("generate path %d to %d\n", i, i+1);
		} else if (i == 0) {
            printf("cancel path %d to %d\n", i, i+1);
            i = i - 1;
        } else {
            printf("delete path %d to %d\n", i-1, i);
            i = i - 2;
            path_to_waypoint.pop_back();
		}
    }

    for (int i = 0; i < size - 1; i++) {
		while (!path_to_waypoint[i].empty()) {
			path_RRT.push_back(path_to_waypoint[i].back());
			path_to_waypoint[i].pop_back();
		}
	}
}
