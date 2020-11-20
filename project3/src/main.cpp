//state definition
#define INIT 0
#define RUNNING 1
#define MAPPING 2
#define PATH_PLANNING 3
#define FINISH -1

#include <unistd.h>
#include <ros/ros.h>
#include <gazebo_msgs/SpawnModel.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/ModelStates.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project2/rrtTree.h>
#include <tf/transform_datatypes.h>
#include <project2/pid.h>
#include <math.h>
#include <pwd.h>

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
int K = 15000;
double MaxStep = 0.5;

//way points
std::vector<point> waypoints;

//path
std::vector<traj> path_RRT;

//control
//std::vector<control> control_RRT;
PID pid_ctrl;

//robot
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;
gazebo_msgs::ModelStatesConstPtr model_states;

//FSM state
int state;

//function definition
void set_waypoints();
void generate_path_RRT();
void callback_state(gazebo_msgs::ModelStatesConstPtr msgs);
void setcmdvel(double v, double w);

int main(int argc, char** argv){
    ros::init(argc, argv, "rrt_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Subscriber gazebo_pose_sub = n.subscribe("/gazebo/model_states",100,callback_state);
    ros::Publisher cmd_vel_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/output",100);
    ros::ServiceClient gazebo_spawn = n.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ros::ServiceClient gazebo_set = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    printf("Initialize topics\n");

    // Load Map

    char* user = getpwuid(getuid())->pw_name;
    map = cv::imread((std::string("/home/") + std::string(user) +
                      std::string("/catkin_ws/src/project3/src/ground_truth_map.pgm")).c_str(), CV_LOAD_IMAGE_GRAYSCALE);

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


    if(!map.data)                              // Check for invalid input
    {
        printf("Could not open or find the image\n");
        return -1;
    }

    // Set Way Points
    set_waypoints();
    printf("Set way points\n");

    // RRT
    generate_path_RRT();
    printf("Generate RRT\n");

    // FSM
    state = INIT;
    bool running = true;
    int look_ahead_idx;
    ros::Rate control_rate(60);

    while(running){
        switch (state) {
            case INIT: {
                look_ahead_idx = 0;
	            printf("path size : %d\n", path_RRT.size());
                //visualize path
	            ros::spinOnce();
                for(int i = 0; i < path_RRT.size(); i++){
		            for(int j = 0; j < model_states->name.size(); j++){
                        std::ostringstream ball_name;
                        ball_name << i;
                	    if(std::strcmp(model_states->name[j].c_str(), ball_name.str().c_str()) == 0){
                            //initialize robot position
                            geometry_msgs::Pose model_pose;
                            model_pose.position.x = path_RRT[i].x;
                            model_pose.position.y = path_RRT[i].y;
                            model_pose.position.z = 0.7;
                            model_pose.orientation.x = 0.0;
                            model_pose.orientation.y = 0.0;
                            model_pose.orientation.z = 0.0;
                            model_pose.orientation.w = 1.0;

                            geometry_msgs::Twist model_twist;
                            model_twist.linear.x = 0.0;
                            model_twist.linear.y = 0.0;
                            model_twist.linear.z = 0.0;
                            model_twist.angular.x = 0.0;
                            model_twist.angular.y = 0.0;
                            model_twist.angular.z = 0.0;

                            gazebo_msgs::ModelState modelstate;
                            modelstate.model_name = ball_name.str();
                            modelstate.reference_frame = "world";
                            modelstate.pose = model_pose;
                            modelstate.twist = model_twist;

                            gazebo_msgs::SetModelState setmodelstate;
                            setmodelstate.request.model_state = modelstate;

                            gazebo_set.call(setmodelstate);
                            continue;
                        }
        		    }
	        
                    gazebo_msgs::SpawnModel model;
                    model.request.model_xml = std::string("<robot name=\"simple_ball\">") +
			        std::string("<static>true</static>") +
                            std::string("<link name=\"ball\">") +
                            std::string("<inertial>") +
                            std::string("<mass value=\"1.0\" />") +
                            std::string("<origin xyz=\"0 0 0\" />") +
                            std::string("<inertia  ixx=\"1.0\" ixy=\"1.0\"  ixz=\"1.0\"  iyy=\"1.0\"  iyz=\"1.0\"  izz=\"1.0\" />") +
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
                            std::string("<sphere radius=\"0.09\"/>") +
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
                    model.request.initial_pose.position.x = path_RRT[i].x;
                    model.request.initial_pose.position.y = path_RRT[i].y;
                    model.request.initial_pose.position.z = 0.7;
                    model.request.initial_pose.orientation.w = 0.0;
                    model.request.initial_pose.orientation.x = 0.0;
                    model.request.initial_pose.orientation.y = 0.0;
                    model.request.initial_pose.orientation.z = 0.0;
                    gazebo_spawn.call(model);
                    ros::spinOnce();
                }
                printf("Spawn path\n");
	
                //initialize robot position
                geometry_msgs::Pose model_pose;
                model_pose.position.x = waypoints[0].x;
                model_pose.position.y = waypoints[0].y;
                model_pose.position.z = 0.3;
                model_pose.orientation.x = 0.0;
                model_pose.orientation.y = 0.0;
                model_pose.orientation.z = 0.0;
                model_pose.orientation.w = 1.0;

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
                ros::Rate(0.33).sleep();

                printf("Initialize ROBOT\n");
                state = RUNNING;
            } break;

            case RUNNING: {
                point goal;
                goal.x = path_RRT[look_ahead_idx].x;
                goal.y = path_RRT[look_ahead_idx].y;
                goal.th = path_RRT[look_ahead_idx].th;
                float ctrl_value = pid_ctrl.get_control(robot_pose, goal);
                float max_steering = 0.3;
                if (fabs(ctrl_value) > max_steering)
                    ctrl_value = max_steering * ctrl_value / fabs(ctrl_value);
                
                //setcmdvel(path_RRT[look_ahead_idx].d,ctrl_value);
                setcmdvel(1.0, ctrl_value);
                cmd_vel_pub.publish(cmd);
                ros::spinOnce();
                control_rate.sleep();
                if (rrtTree::distance(path_RRT[look_ahead_idx], robot_pose) < (look_ahead_idx == path_RRT.size()-1 ? 0.2 : 0.5) 
                    && look_ahead_idx < path_RRT.size()) {
                    look_ahead_idx++;
                    pid_ctrl.reset();
                }
                if(look_ahead_idx == path_RRT.size()) state = FINISH;
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
	//TODO
	// printf("Start generate_path_RRT()\n");
	int size = waypoints.size();
	std::vector<std::vector<traj>> path_to_waypoint;
	// printf("waypoints.size() : %d \n",size);
	for (int i = 0; i < size - 1; i++) {
		// printf("Start of For Loop, i : %d\n",i);
		rrtTree Tree = rrtTree(waypoints[i], waypoints[i + 1], map, map_origin_x, map_origin_y, res, margin);
		// printf("After Called rrtTree Constructor\n");
		Tree.generateRRT(world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
		// printf("After Called generateRRT\n");
		std::vector<traj> temp_path = Tree.backtracking_traj();
		// printf("After Called path_to_waypoint\n");

		bool well_made = rrtTree::distance(temp_path.front(), waypoint[i + 1]) < 0.5; // path가 제대로 생기지 않았는지 판단



		std::vector<traj> start_waypoint;
		traj waypoint;
		waypoint.x = waypoints[i + 1].x;
		waypoint.y = waypoints[i + 1].y;
		waypoint.th = waypoints[i + 1].th;
		waypoint.d = 0.325;
		waypoint.alpha = 0;
		start_waypoint.push_back(waypoint);
		start_waypoint.insert(start_waypoint.end(), temp_path.begin(), temp_path.end()); // start_waypoint는 원래 목적지 한 점만 포함, temp_path는 목적지 미포함 출발지까지 역순서로 되어있음.
		//결과적으로 start_waypoint는 목적지포함 출발지까지 역순으로 갖고있는 vector가 됨
		// path_to_waypoint.push_back(waypoints[i]);
		// printf("i : %d\n", i);
		// printf("waypoints[i+1].th : %f \n", waypoints[i+1].th);
		if (well_made) {
			printf("Well Made, i : %d\n", i);
			if (!path_to_waypoint.empty()) {
				waypoints[i + 1].th = temp_path[1].th; // 잘 만들어졌다면 입장각도를 복사함. temp_path[0]은 위에서 concat하면서 waypoint(정답)을 가리키므로 [1]이 사실상 rrtTree에서 정답과 가장 가까웠던 목표지점.
			}
			path_to_waypoint.push_back(temp_path);
		}
		else {
			printf("Trash & Bad, i : %d\n", i);
			if (i == 0) i = i - 1;
			else {
				i = i - 2;
				path_to_waypoint.pop_back();
			}
		}
		/////////////////////////// FOR DEBUGGING /////////////////////////////////

		std::vector<traj> path_reversed;
		while (!start_waypoint.empty()) {
			path_reversed.push_back(start_waypoint.back());
			start_waypoint.pop_back();
		}
		printf("Before visualize\n, current start : %d, goal : %d\n", i, i + 1);
		Tree.visualizeTree(path_reversed);
		printf("After visualize\n");
		Tree.visualizeTree(path_reversed);
		getchar();
		/////////////////////////// FOR DEBUGGING /////////////////////////////////
	}

	for (int i = 0; i < size - 1; i++) {
		while (!path_to_waypoint[i].empty()) {
			path_RRT.push_back(path_to_waypoint[i].back());
			path_to_waypoint[i].pop_back();
		}
	}

	printf("End of generate_path_RRT, showing total path\n");
	Tree.visualizeTree(path_reversed);
	printf("After visualize\n");
	Tree.visualizeTree(path_reversed);
	getchar();
}

void set_waypoints()
{
    point waypoint_candid[5];
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 12.0;
    waypoint_candid[1].x = 2.0;
    waypoint_candid[1].y = 12.0;
    waypoint_candid[2].x = 3.5;
    waypoint_candid[2].y = -10.5;
    waypoint_candid[3].x = -2.0;
    waypoint_candid[3].y = -12.0;
    waypoint_candid[4].x = -3.5;
    waypoint_candid[4].y = 10.0;

    int order[] = {0,1,2,3,4};
    int order_size = 5;

    for(int i = 0; i < order_size; i++){
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}


void callback_state(gazebo_msgs::ModelStatesConstPtr msgs){
    model_states = msgs;
    for(int i; i < msgs->name.size(); i++){
        if(std::strcmp(msgs->name[i].c_str(),"racecar") == 0){
            robot_pose.x = msgs->pose[i].position.x;
            robot_pose.y = msgs->pose[i].position.y;
            robot_pose.th = tf::getYaw(msgs->pose[i].orientation);
        }
    }
}

void setcmdvel(double vel, double deg){
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}
