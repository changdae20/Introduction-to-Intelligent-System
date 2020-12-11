#include <iostream>
#include <cstdlib>
#include <climits>
#include <ctime>
#include <cmath>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#ifndef POINT_H
#define POINT_H
#include <project2/point.h>
#include <project2/control.h>
#include <project2/traj.h>
#endif

const int N = 15000;

class rrtTree
{
private:
    struct node
    {
        int idx;
        point rand;
        point location;
        int idx_parent;
	    double alpha;
	    double d;
    }*root;

    int count;
    point x_init, x_goal;
    cv::Mat map;
    cv::Mat map_original;
    double map_origin_x, map_origin_y;
    double res;
    node *ptrTable[N];

    cv::Mat addMargin(cv::Mat map, int margin);
    void addVertex(point x_new, point x_rand, int idx_near, double alpha, double d);
    int nearestNeighbor(point x_rand, double MaxStep);
    int nearestNeighbor(point x_rand);
    bool isCollision(point x1, point x2, double d, double R);
    point randomState(double x_max, double x_min, double y_max, double y_min);
    int randompath(double *out, point x_near, point x_rand, double MaxStep);
    // int nearestNeighbor(int, int);

public:
    rrtTree();
    rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x, double map_origin_y, double res, int margin);
    rrtTree(point x_init, point x_goal);
    ~rrtTree();

    void visualizeTree();
    void visualizeTree(std::vector<traj> path);
    int generateRRT(double x_max, double x_min, double y_max, double y_min, int K, double MaxStep);
    std::vector<traj> backtracking_traj();

    bool isCollision(traj x1, traj x2, double d, double R);

    static double distance(const point&, const point&);
    static double distance(const traj&, const traj&);
    static double distance(const traj&, const point&);
    static double thetaModulo(const double&, const double&);
    static point traj2point(const traj&);
    static traj point2traj(const point&);
    static traj predict_point(traj, traj, double);
};
