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
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
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
    cv::Rect imgROI((int)Res*200,(int)Res*200,(int)Res*400,(int)Res*400);
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha, double d) {
    //TODO
    ++count;
    ptrTable[count] = new node;
    ptrTable[count]->idx = count;
    ptrTable[count]->rand = x_rand;
    ptrTable[count]->location = x_new;
    ptrTable[count]->idx_parent = idx_near;
    ptrTable[count]->alpha = alpha;
    ptrTable[count]->d = d;
}

int rrtTree::generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep) {
    //TODO
    for(i=0;i<K;i++){
        point x_rand = randomState(x_max,x_min,y_max,y_min);
        double out[5];
        int x_near = nearestNeighbor(x_rand, MaxStep);
        int valid = randompath(out, ptrTable[x_near], x_rand, MaxStep);
        if(valid == 1){
            point x_new;
            x_new.x = out[0];
            x_new.y = out[1];
            x_new.th = out[2];
            addVertex(x_new, x_rand, x_near, out[3], out[4]);
        }
    }
}

point rrtTree::randomState(double x_max, double x_min, double y_max, double y_min) {
    //TODO
    double max = 32767;
    double x_rand = x_min + rand()%((int)(x_max-x_min)) + ((double)rand())/max;
    double y_rand = y_min + rand()%((int)(y_max-y_min)) + ((double)rand())/max;
    
    point newpoint;
    newpoint.x = x_rand;
    newpoint.y = y_rand;
    return newpoint;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    //TODO
    double max_beta = MaxStep * tan(max_alpha) / L;
    int min_idx = 0;
    for (int i = 1; i <= count; ++i) {
        if (distance(x_rand, ptrTable[i]->location) < distance(x_rand, ptrTable[min_idx]->location)
            && fabs(thetaModulo(ptrTable[i]->location.th, -x_rand.th)) < max_beta)
            min_idx = i;
    }
    return min_idx;
}

int rrtTree::nearestNeighbor(point x_rand) {
    //TODO
    int index = 0;
    double min_distance = distance(x_rand, root->location);
    for(int i = 1; i <= count; ++i) {
        if(min_distance > distance(x_rand, ptrTable[i]->location)) {
            min_distance = distance(x_rand, ptrTable[i]->location);
            index = i;
        }
    }
    return index;
}

int rrtTree::randompath(double *out, point x_near, point x_rand, double MaxStep) {
    //TODO
    int sample_size = 30;
    double[30] d_array; alpha_array;
    point[30] sample_point;
    int min_distance_idx = 0;
    for(int i=0; i<30; i++){
        d_array[i] = randomState(MaxStep,0,0,0).x;
        alpha_array[i] = randomState(max_alpha,-max_alpha,0,0).x;
        double radius = L / tan(fabs(alpha_array[i]));
        double beta = d_array[i] / radius;
        sample_point[i].x = x_near.x - radius*sin(x_near.th) + radius*sin(x_near.th + beta);
        sample_point[i].x = x_near.x + radius*cos(x_near.th) - radius*cos(x_near.th + beta);
        sample_point[i].th = x_near.th + beta;
    }
    double min_distance = distance(x_rand,sample_point[0]);
    for(int i=1; i<30; i++){
        if(distance(x_rand,sample_point[i])<min_distance){
            min_distance = distance(x_rand,sample_point[i]);
            min_distance_idx = i;
        }
    }
    out[0] = sample_point[min_distance_idx].x;
    out[1] = sample_point[min_distance_idx].y;
    out[2] = sample_point[min_distance_idx].th;
    out[3] = alpha_array[min_distance_idx];
    out[4] = d_array[min_distance_idx];

    return (int)isCollision(x_near, sample_point[min_distance_idx], d_array[min_distance_idx], L / tan(fabs(alpha_array[i])));
}

bool rrtTree::isCollision(point x1, point x2, double d, double R) {
    //TODO
    int i1 = x1.x / res + map_origin_x;
    int i2 = x2.x / res + map_origin_x;
    int j1 = x1.y / res + map_origin_y;
    int j2 = x2.y / res + map_origin_y;

    if (i1 > i2) {
        int tmp = i1; i1 = i2; i2 = tmp;
        tmp = j1; j1 = j2; j2 = tmp;
    }
    for (int i = i1; i < i2; ++i) {
        int j = (j2 - j1) / (i2 - i1) * (i - i1) + j1;
        if (map.at<uchar>(i, j) < 125) return true;
    }

    if (j1 > j2) {
        int tmp = j1; j1 = j2; j2 = tmp;
        tmp = i1; i1 = i2; i2 = tmp;
    }
    for (int j = j1; j < j2; ++j) {
        int i = (i2 - i1) / (j2 - j1) * (j - j1) + i1;
        if (map.at<uchar>(i, j) < 125) return true;
    }

    return false;
}

std::vector<traj> rrtTree::backtracking_traj(){
    //TODO
    node current_node = ptrTable[nearestNeighbor(this->x_goal)];
    std::vector<traj> path;
    while(currnet_node != this->x_init){
        traj current_node_traj{current_node.location.x, current_node.location.y, current_node.location.th, current_node.d, current_node.alpha};
        path.push_back(current_node_traj);
        current_node_traj = ptrTable[nearestNeighbor(current_node.idx_parent)];
    }
    return path;
}

double rrtTree::distance(point p1, point p2) {
    return hypot(p2.x - p1.x, p2.y - p1.y);
}

double rrtTree::thetaModulo(double th1, double th2) {
    return -M_PI + fmod(3 * M_PI + th1 + th2, 2 * M_PI);
}