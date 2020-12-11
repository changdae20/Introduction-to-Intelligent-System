#include "rrtTree.h"
#include <unistd.h>
#include <ros/ros.h>
#define PI 3.14159265358979323846

double max_alpha = 0.2;
double L = 0.325;

rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::rrtTree(point x_init, point x_goal) {
    this->x_init = x_init;
    this->x_goal = x_goal;

    std::srand(std::time(NULL));
    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
    root->alpha = 0;
    root->d = 0;
}

rrtTree::~rrtTree(){
    for (int i = 1; i <= count; i++) {
        delete ptrTable[i];
    }
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map.clone();
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;
    std::srand(std::time(NULL));

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = NULL;
    root->location = x_init;
    root->rand = x_init;
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree(){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);
    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	    for(int j = 0; j < 10; j++) {
	        double alpha = this->ptrTable[i]->alpha;
	        double d = this->ptrTable[i]->d;
	        double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	        double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	        double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	    }
    }
    cv::namedWindow("Mapping");
    // cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    // cv::Rect imgROI((int)Res*0,(int)Res*0,(int)Res*540,(int)Res*179);
    // cv::Rect imgROI((int)Res*0,(int)Res*0,(int)Res*408,(int)Res*188);
    cv::Rect imgROI((int)0,(int)0,(int)Res*map.cols,(int)Res*map.rows);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::visualizeTree(std::vector<traj> path){
    int thickness = 1;
    int lineType = 8;
    int idx_parent;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(imgResult, cv::Point((int)(Res*(path[0].y/res + map_origin_y)), (int)(Res*(path[0].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult, cv::Point((int)(Res*(path[path.size()-1].y/res + map_origin_y)), (int)(Res*(path[path.size()-1].x/res + map_origin_x))), radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for(int i = 1; i < this->count; i++) {
        idx_parent = this->ptrTable[i]->idx_parent;
	    for(int j = 0; j < 10; j++) {
	        double alpha = this->ptrTable[i]->alpha;
	        double d = this->ptrTable[i]->d;
	        double p1_th = this->ptrTable[idx_parent]->location.th + d*j/10*tan(alpha)/L;
            double p2_th = this->ptrTable[idx_parent]->location.th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
	        double p1_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x + L/tan(alpha)*(sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
	        double p2_y = this->ptrTable[idx_parent]->location.y + L/tan(alpha)*(cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	    }
    }

    thickness = 3;
    for(int i = 1; i < path.size(); i++) {
	    for(int j = 0; j < 10; j++) {
	        double alpha = path[i].alpha;
	        double d = path[i].d;
	        double p1_th = path[i-1].th + d*j/10*tan(alpha)/L; // R = L/tan(alpha)
            double p2_th = path[i-1].th + d*(j+1)/10*tan(alpha)/L;
            double p1_x = path[i-1].x + L/tan(alpha)*(sin(p1_th) - sin(path[i-1].th));
	        double p1_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p1_th));
            double p2_x = path[i-1].x + L/tan(alpha)*(sin(p2_th) - sin(path[i-1].th));
	        double p2_y = path[i-1].y + L/tan(alpha)*(cos(path[i-1].th) - cos(p2_th));
            x1 = cv::Point((int)(Res*(p1_y/res + map_origin_y)), (int)(Res*(p1_x/res + map_origin_x)));
            x2 = cv::Point((int)(Res*(p2_y/res + map_origin_y)), (int)(Res*(p2_x/res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
	    }
    }

    cv::namedWindow("Mapping");
    //cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::Rect imgROI((int)0,(int)0,(int)Res*map.cols,(int)Res*map.rows);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d) {
    //TODO
    ptrTable[count] = new node;
    ptrTable[count]->idx = count;
    ptrTable[count]->rand = x_rand;
    ptrTable[count]->location = x_new;
    ptrTable[count]->idx_parent = idx_near;
    ptrTable[count]->alpha = alpha;
    ptrTable[count]->d = d;
    ++count;
}

int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    //TODO
    int cnt = 0;
    // int idx_of_min = 0;
    // while ((cnt + 1) * K <= N) {
    //     printf("cnt: %d\n", cnt);
    //     double dist = distance(x_goal, ptrTable[idx_of_min]->location);
    //     if (dist < 0.5) break;
        while (count < (cnt + 1) * K){
            point x_rand = randomState(x_max,x_min,y_max,y_min);
            double out[5];
            int x_near = nearestNeighbor(x_rand, MaxStep);
            if(randompath(out, ptrTable[x_near]->location, x_rand, MaxStep)) {
                point x_new;
                x_new.x = out[0];
                x_new.y = out[1];
                x_new.th = out[2];
                addVertex(x_new, x_rand, x_near, out[3], out[4]);
            }
        }
    //     idx_of_min = nearestNeighbor(x_goal);
    //     // if (dist - distance(x_goal, ptrTable[idx_of_min]->location) < 0.5) break;
    //     ++cnt;
    // }
    return 0;
}

point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    //TODO
    double x_rand = x_min + (x_max-x_min)*rand()/RAND_MAX;
    double y_rand = y_min + (y_max-y_min)*rand()/RAND_MAX;
    
    point newpoint;
    newpoint.x = x_rand;
    newpoint.y = y_rand;

    return newpoint;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    //TODO
    double max_beta = MaxStep * tan(max_alpha) / L;
    int min_idx = 0;
    for (int i = 1; i < count; ++i) {
        float relative_angle = atan2(x_rand.y - ptrTable[i]->location.y, x_rand.x - ptrTable[i]->location.x);
        if (distance(x_rand, ptrTable[i]->location) < distance(x_rand, ptrTable[min_idx]->location)
            && fabs(thetaModulo(ptrTable[i]->location.th, -relative_angle)) < max_beta)
            min_idx = i;
    }
    return min_idx;
}

int rrtTree::nearestNeighbor(point x_rand) {
    //TODO
    int index = 0;
    double min_distance = distance(x_rand, root->location);
    for(int i = 1; i < count; ++i) {
        if(min_distance > distance(x_rand, ptrTable[i]->location)) {
            min_distance = distance(x_rand, ptrTable[i]->location);
            index = i;
        }
    }
    return index;
}

// int rrtTree::nearestNeighbor(int idx_of_min, int idx_of_start) {
//     //TODO
//     int index = idx_of_min;
//     double min_distance = distance(x_goal, ptrTable[index]->location);
//     for(int i = idx_of_start; i < count; ++i) {
//         if(min_distance > distance(x_goal, ptrTable[i]->location)) {
//             min_distance = distance(x_goal, ptrTable[i]->location);
//             index = i;
//         }
//     }
//     return index;
// }

int rrtTree::randompath(double *out, point x_near, point x_rand, double MaxStep) {
    //TODO
    int sample_size = 100;
    double d_array[sample_size]; 
    double alpha_array[sample_size];
    point sample_point[sample_size];
    int min_distance_idx = 0;
    for(int i=0; i<sample_size; i++){
        d_array[i] = L + (MaxStep-L)*rand()/RAND_MAX;
        alpha_array[i] = -max_alpha + (2*max_alpha)*rand()/RAND_MAX;
        double radius = L / tan(alpha_array[i]);
        double beta = d_array[i] / radius;
        sample_point[i].x = x_near.x - radius*sin(x_near.th) + radius*sin(x_near.th + beta);
        sample_point[i].y = x_near.y + radius*cos(x_near.th) - radius*cos(x_near.th + beta);
        sample_point[i].th = thetaModulo(x_near.th , beta);
    }
    double min_distance = distance(x_rand,sample_point[0]);
    for(int i=1; i<sample_size; i++){
        if(distance(x_rand,sample_point[i])<min_distance ){
            min_distance = distance(x_rand,sample_point[i]);
            min_distance_idx = i;
        }
    }
    out[0] = sample_point[min_distance_idx].x;
    out[1] = sample_point[min_distance_idx].y;
    out[2] = sample_point[min_distance_idx].th;
    out[3] = alpha_array[min_distance_idx];
    out[4] = d_array[min_distance_idx];

    return 1-(int)isCollision(x_near, sample_point[min_distance_idx], d_array[min_distance_idx], L / tan(alpha_array[min_distance_idx]));
}

bool rrtTree::isCollision(point x1, point x2, double d, double R) {
    //TODO
    double x_c = x1.x - R * sin(x1.th);
    double y_c = x1.y + R * cos(x1.th);
    double beta = d / R;
    double dbeta = res / R;
    for (int i = 1; i < d / res; ++i) {
        double x_temp = x_c + R * sin(x1.th + dbeta * i);
        double y_temp = y_c - R * cos(x1.th + dbeta * i);
        int i1_temp = x_temp / res + map_origin_x;
        int j1_temp = y_temp / res + map_origin_y;
        if (map.at<uchar>(i1_temp, j1_temp) < 125) return true;
    }
    return false;
}

bool rrtTree::isCollision(traj x1, traj x2, double d, double R) {
    //TODO
    double x_c = x1.x - R * sin(x1.th);
    double y_c = x1.y + R * cos(x1.th);
    double beta = d / R;
    double dbeta = res / R;
    for (int i = 1; i < d / res; ++i) {
        double x_temp = x_c + R * sin(x1.th + dbeta * i);
        double y_temp = y_c - R * cos(x1.th + dbeta * i);
        int i1_temp = x_temp / res + map_origin_x;
        int j1_temp = y_temp / res + map_origin_y;
        if (map.at<uchar>(i1_temp, j1_temp) < 125) return true;
    }
    return false;
}


std::vector<traj> rrtTree::backtracking_traj(){
    //TODO
    std::vector<traj> path;
    for (node current_node = *ptrTable[nearestNeighbor(x_goal)]; current_node.location != x_init; current_node = *ptrTable[current_node.idx_parent]) {
        traj current_node_traj = traj(current_node.location.x, current_node.location.y, current_node.location.th, current_node.d, current_node.alpha);
        path.push_back(current_node_traj);
    }
    return path;
}

double rrtTree::distance(const point& p1, const point& p2) {
    return hypot(p2.x - p1.x, p2.y - p1.y);
}

double rrtTree::distance(const traj& p1, const traj& p2) {
    return hypot(p2.x - p1.x, p2.y - p1.y);
}

double rrtTree::distance(const traj& p1, const point& p2) {
    return hypot(p2.x - p1.x, p2.y - p1.y);
}

double rrtTree::thetaModulo(const double& th1, const double& th2) {
    return -M_PI + fmod(3 * M_PI + th1 + th2, 2 * M_PI);
}

point rrtTree::traj2point(const traj& a) {
    return point(a.x, a.y, a.th);
}

traj rrtTree::point2traj(const point& a) {
	return traj(a.x, a.y, a.th, 0, 0);
}

traj rrtTree::predict_point(traj origin, traj goal, double d) {
    traj ret;
    double R = L / tan(goal.alpha);
    double beta = d / R;
    ret.x = origin.x - R*sin(origin.th) + R*sin(origin.th+beta);
    ret.y = origin.y + R*cos(origin.th) - R*cos(origin.th+beta);
    return ret;
}
