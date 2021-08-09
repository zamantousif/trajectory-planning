#include <iostream>
#include "gnuplot-iostream.h"
#include "trajectory.hpp"

//#include "test1.h"


// function to compute distance between two points in cartesian plane
double distance(double x1, double y1, double x2, double y2){
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

// function to find the waypoint closest to a given point on the plane
int findWaypoint(double x, double y, const vector<pair<double, double>>& v)
{
    int waypoint = 0;
    int refTrajX, refTrajY;
    double dist;
    double closestDist = LARGEVAL;
    // iterate over the vector to find the shortest distance between the waypoints and the given point on the cartesian plane
    for(unsigned int i = 0; i < v.size(); ++i){
        refTrajX = v[i].first;
        refTrajY = v[i].second;
        dist = distance(x, y, refTrajX, refTrajY);
        // store the closest distance and the waypoint number
        if (dist < closestDist){
            closestDist = dist;
            waypoint = i;
        }
    }
    return waypoint;
}


// task #1
// class member function to plot a continuous reference trajectory given a set of points in the cartesian plane
int Car::plotRefTraj(const vector<pair<double, double>>& v){
    // gnuplot object
    Gnuplot gp;
    // plot a continuous reference trajectory using the XY coordinates
    gp << "plot" << gp.file1d(v, "myplot.dat") << "with lines title 'Reference Trajectory' " << endl;

    return 0;
}

// task #2
// class member function to convert point in cartesian coordinate system (X,Y) to the frenet coordinate system (Latitude, Longitude)
vector<pair<double, double>> Car::cartesianToFrenet(double x, double y, const vector<pair<double, double>>& v){
    //double frenet_latitude = 0.0;
    //double frenet_longitude= 0.0;

    double ax, ay, bx, by, x1, y1, x2, y2;
    double scaling_fac;
    double frenet_long = 0;
    vector <pair<double, double>> frenet_vec;
    int waypoint_num;
    bool leftX = false;
    bool leftY = false;

    waypoint_num = findWaypoint(x, y, v);
    x2 = v[waypoint_num].first;
    y2 = v[waypoint_num].second;
    x1 = v[waypoint_num-1].first;
    y1 = v[waypoint_num-1].second;

    // compute x and y components of vector a between (x,y) and (x1,y1)
    ax = x - x1;
    ay = y - y1;
    // compute x and y components of vector b between (x2,y2) and (x1,y1)
    bx = x2 - x1;
    by = y2 - y1;
    // next step is to compute the projection of (x,y) on the reference trajectory
    // vector b lies on the reference trajectory and therefore kb.(a-kb) = 0
    // k = some scaling factor for vector b
    // k = a.b/b.b where . is the vector dot product
    scaling_fac = (ax*bx + ay*by)/(bx*bx + by*by);
    double projection_x = scaling_fac*bx;
    double projection_y = scaling_fac*by;

    // latitude on the frenet coordinate system
    double frenet_lat = distance(x, y, projection_x, projection_y);

    // check if the given point (x,y) is on the left/right of the reference trajectory
    if(x > projection_x)
        leftX = true;

    if(y > projection_y)
        leftY = true;

    if(leftX && leftY){
        frenet_lat *= -1;
    }

    // longitude on the frenet coordinate system
    // compute distance until the waypoint closest to the given point (x,y) and add up the distance of this waypoint to the point (x,y)
    for(int i = 0; i < waypoint_num; ++i){
        frenet_long += distance(v[i].first, v[i].second, v[i+1].first, v[i+1].second);
    }

    frenet_long += distance(x, y, v[waypoint_num].first, v[waypoint_num].second);

    // pair frenet_lat and frenet_long and store the pair in vector
    frenet_vec.push_back(make_pair(frenet_lat, frenet_long));

    return frenet_vec;
}
