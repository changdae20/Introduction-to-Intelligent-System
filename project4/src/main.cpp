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
int margin = 5;
int K = 1500;
double MaxStep = 2.0;
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
                              std::string("/catkin_ws/src/project4/src/slam_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org,map);
            cv::flip(map,map,1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range/2.0 - 0.5;
            map_origin_y = map_y_range/2.0 - 0.5;
            world_x_min = -4.5;
            world_x_max = 4.5;
            world_y_min = -13.5;
            world_y_max = 13.5;
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
            //TODO 1

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
    std::srand(std::time(NULL));
    point waypoint_candid[5];
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 12.0;

    cv::Mat map_margin = map.clone();
    int jSize = map.cols; // the number of columns
    int iSize = map.rows; // the number of rows

    for (int i = 0; i < iSize; i++) {
        for (int j = 0; j < jSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - waypoint_margin; k <= i + waypoint_margin; k++) {
                    for (int l = j - waypoint_margin; l <= j + waypoint_margin; l++) {
                        if (k >= 0 && l >= 0 && k < iSize && l < jSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    //TODO 2

    int arr_x[3] = {1,1,-1};
    int arr_y[3] = {1,-1,-1};
    for (int i = 0 ; i < 3 ; ++i){
        int x_index;
        int y_index;
        do {
            x_index = map_origin_x + arr_x[i] * iSize * ((double)rand() / RAND_MAX) / 2;
            y_index = map_origin_y + arr_y[i] * jSize * ((double)rand() / RAND_MAX) / 2;
        } while(map_margin.at<uchar>(x_index, y_index) < 125);
        waypoint_candid[i+1].x = res*(x_index-map_origin_x);
        waypoint_candid[i+1].y = res*(y_index-map_origin_y);
    }

    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 12.0;

    int order[] = {0,1,2,3,4,1,2,3};
    int order_size = 8;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void generate_path_RRT()
{
    //TODO 1
    int size = waypoints.size();
    int max_failure = 70;
    int failed[size] = {0, };
    time_t start_time = time(NULL);
	std::vector< std::vector<traj> > path_to_waypoint;
    std::vector<point> last_points = waypoints;

	for (int i = 0; i < size - 1; i++) {
		rrtTree Tree = rrtTree(last_points[i], waypoints[i + 1], map, map_origin_x, map_origin_y, res, margin);

        Tree.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);

        // Tree.visualizeTree(); Tree.visualizeTree(); getchar();

		std::vector<traj> start_waypoint = Tree.backtracking_traj();
		start_waypoint.push_back(rrtTree::point2traj(last_points[i]));
		bool well_made = rrtTree::distance(start_waypoint.front(), waypoints[i + 1]) < 0.5;

        if (well_made) {
			last_points[i + 1] = rrtTree::traj2point(start_waypoint.front());
			path_to_waypoint.push_back(start_waypoint);
            printf("generate path %d to %d\n\n", i, i+1);
            failed[i+1] = 0;
		} else {
            if (failed[i+1] + 1 >= max_failure) {
                printf("Too much failure to plan path, it'll give you the best result only until waypoint %d\n", i);
                break;
            }
            ++failed[i+1];
            printf("failed to go to waypoint %d (count: %d / %d)\n", i+1, failed[i+1], max_failure);
            if (i <= 0) {
                printf("cancel path %d to %d\n\n", i, i+1);
                i = i - 1;
            } else {
                printf("delete path %d to %d\n\n", i-1, i);
                i = i - 2;
                path_to_waypoint.pop_back();
            }
        }
        // if (time(NULL) - start_time > 210) {
        //     printf("Too much time to generate the path\n");
        //     break;
        // }
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
