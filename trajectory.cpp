//#include "trajectory.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <string>
#include <cmath>
#include <boost/lexical_cast.hpp>
#include "gnuplot-iostream.h"
using boost::lexical_cast;
using std::string;
using namespace std;
// use full precision for double
typedef numeric_limits<double> dbl;

// define constants
const double PI = 3.14159265;
const double LARGEVAL = 1000000.0;

// function to compute distance between two points in cartesian plane
double getDistance(double x1, double y1, double x2, double y2){
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

// function to compute cross-product of two vectors a and b in 2D plane
double crossProduct(double ax, double ay, double bx, double by){
    return (ax*by - ay*bx);
}

// function to compute dot-product of two vectors a and b in 2D plane
double dotProduct(double ax, double ay, double bx, double by){
    return (ax*bx + ay*by);
}

// function to compute the angle between two vectors a and b in 2D plane
double getAngle(double x, double y){
    return atan2(y, x) * (180.0 / PI);
}

// double getAngle(double ax, double ay, double bx, double by){
//     double theta = 0.0;
//     theta = acos((ax*bx + ay*by)/(sqrt(ax*ax + ay*ay) * sqrt(bx*bx + by*by))) * (180.0 / PI);
//
//     return theta;
// }


// function to find the waypoint on the reference trajectory that is closest to a given point on the same plane
int findWaypoint(double x, double y, const vector<pair<double, double>>& v){
    int waypoint = 0;
    int refTrajX, refTrajY;
    double dist;
    double closestDist = LARGEVAL;
    // iterate over the vector to find the shortest distance between the waypoints and a given point on the cartesian plane
    for(int i = 0; i < v.size(); ++i){
        refTrajX = v[i].first;
        refTrajY = v[i].second;
        dist = getDistance(x, y, refTrajX, refTrajY);
        // store the closest distance and the waypoint number
        if (dist < closestDist){
            closestDist = dist;
            waypoint = i;
        }
    }

    cout << "Nearest waypoint is " << waypoint << endl;
    return waypoint;
}

// C++ template to print the contents of the vector
template <typename d1, typename d2>
ostream& operator<<(ostream& os, const vector<d1, d2>& v)
{
    // iterate over the vector v to print each coordinate pair
    for(int i = 0; i < v.size(); ++i)
        os << "(" << v[i].first << ", " << v[i].second << ")";

    return os;
}

// Class for the ego car
class Car{
    public:

    int plotRefTraj(const vector<pair<double, double>>& v);
    vector<pair<double, double>> cartesianToFrenet(double x, double y, const vector<pair<double, double>>& v);

};

// Task #1
// class member function to plot a continuous reference trajectory given a set of points in the cartesian plane
int Car::plotRefTraj(const vector<pair<double, double>>& v){
    // gnuplot object
    Gnuplot gp;
    // plot a continuous reference trajectory using the XY coordinates
    gp << "plot" << gp.file1d(v, "myplot.dat") << "with lines title 'Reference Trajectory' " << endl;

    return 0;
}


// Task #2
// class member function to convert point in cartesian coordinate system (X,Y) to the frenet coordinate system (Latitude, Longitude)
vector<pair<double, double>> Car::cartesianToFrenet(double x, double y, const vector<pair<double, double>>& v){
    double vx, vy, sx, sy, x1, y1, x2, y2;
    double frenet_long = 0.0;
    vector <pair<double, double>> frenet_vec;
    int waypoint_num;
    int waypoint_max;
    bool onLeft = false;
    bool onRight = false;
    bool onLine = false;

    // waypoint numbering ranges from [0] to [v.size() minus 1]
    waypoint_num = findWaypoint(x, y, v);
    waypoint_max = v.size()-1;

    // nearest waypoint to given point (x,y)
    x1 = v[waypoint_num].first;
    y1 = v[waypoint_num].second;


    if (waypoint_num == 0){
        // if the nearest waypoint is the start point of trajectory then use the next waypoint for the tangential vector and invert the sign to keep tangent in directing of increasing longitude
        x2 = -1*v[waypoint_num+1].first;
        y2 = -1*v[waypoint_num+1].second;
    }
    else{
        // previous waypoint to (x1,y1)
        x2 = v[waypoint_num-1].first;
        y2 = v[waypoint_num-1].second;
    }

    cout << "Nearest waypoint is " << endl;
    cout << "x1 = " << x1 << " " << "y1 = " << y1 << endl;
    cout << "Waypoint previous to the nearest waypoint is " << endl;
    cout << "x2 = " << x2 << " " << "y2 = " << y2 << endl;

    // compute x and y components of vector v between (x,y) and the nearest waypoint (x1,y1) on the reference trajectory
    vx = x - x1;
    vy = y - y1;
    cout << "vector V = " << vx << " " << vy << endl;
    // compute x and y components of vector S between nearest waypoint (x1,y1) and the next waypoint (x2,y2) both located on the reference trajectory
    sx = x1 - x2;
    sy = y1 - y2;
    cout << "vector S = " << sx << " " << sy << endl;

    double angle = getAngle(sy, sx);
    cout << "theta from atan2 = " << angle << endl;

    // latitude on the frenet coordinate system
    double dist_v = getDistance(x, y, x1, y1);
    double frenet_lat = dist_v * cos(angle);

    // Sign of the cross Product of vector s and vector v determines the position of the point (x,y) with respect to the vector s on reference trajectory
    double cp = crossProduct(sx, sy, vx, vy);
    cout << "Cross Product = " << cp << endl;
    // Check if the given point (x,y) is on the left/right of the reference trajectory and update frenet latitude
    if(cp < 0)
        onRight = true;
    else if(cp > 0)
        onLeft = true;
    else
        onLine = true;

    // print appropriate messages for debugging
    if(onLeft){
        // frenet latitude must be negative
        frenet_lat *= -1;
        cout << "The given point lies to the left side of the reference trajectory" << endl;
    }
    else if(onRight){
        // frenet latitude must be positive
        cout << "The given point lies to the right side of the reference trajectory" << endl;
    }
    else if(onLine){
        cout << "The given point lies on the reference trajectory" << endl;
    }
    else{
        cout << "Check the given point, it looks like it is nowhere!" << endl;
    }

    cout << "frenet latitude = " << frenet_lat << endl;

    // longitude on the frenet coordinate system
    // compute distance until the waypoint nearest to the given point (x,y)
    for(int i = 0; i < waypoint_num; ++i){
        frenet_long += getDistance(v[i].first, v[i].second, v[i+1].first, v[i+1].second);
    }
    // add up the parallel distance of the given point (x,y) from nearest waypoint (x1,y1) for more accuracy
    double long_acc = dist_v * sin(angle);
    cout << "longitude accuracy = " << long_acc << endl;
    // Ignore this accuracy at the start and end points of the trajectory
    if ((waypoint_num != 0) && (waypoint_num != waypoint_max))
        frenet_long += long_acc;

    // pair frenet_lat and frenet_long and store the pair in vector
    frenet_vec.push_back(make_pair(frenet_lat, frenet_long));

    return frenet_vec;
}

// main
int main(){

    // Car object
    Car car1;
    vector<pair<double, double>> pts_XY;
    vector<pair<double, double>> pt_frenet;
    double X = 0.0;
    double Y = 0.0;

    // begin file operation
    // open file to read the coordinates before plotting them
    ifstream file("input.txt");
    // scan each line in the file to update the points to the respective array
    if(file.is_open()){
        string line;
        int numline = 1;
        while(getline(file, line)){
            // odd lines contain X while even lines contain Y
            if(numline % 2 == 1){
                X = stod(line);
            }
            else{
                Y = stod(line);
                // make X,Y pair and store in a vector
                pts_XY.push_back(make_pair(X,Y));
            }
            numline++;
        }
    }
    file.close();
    // end file operation

    // print the XY coordinate pairs with maximum precision
    cout << std::setprecision(numeric_limits<double>::max_digits10) << pts_XY << endl;
    // alternative way to print out the values with best precision match
    // cout << lexical_cast<string>(pts_XY) << endl;

    // Task 1 : plot the reference trajectory from the given Cartesian coordinates
    car1.plotRefTraj(pts_XY);


    // Test Inputs:

    // start point on reference trajectory
    // double x = 0.0;
    // double y = 0.119615256786;
    // 2nd point on reference trajectory
    // double x = 1.99999809265;
    // double y = 0.116609536111;
    // end point on reference trajectory
    // double x = 41.9993400574;
    // double y = -0.101188264787;
    // left of reference trajectory
    // double x = 25.0;
    // double y = 0.15;
    // double x = 40.0;
    // double y = -0.05;
    // right of reference trajectory
    double x = 15.0;
    double y = 0.0;
    // double x = 3.994565;
    // double y = 0.02345;
    // corner case with zero input
    // double x = 0.0;
    // double y = 0.0;
    // corner case with large input
    // double x = 100000000.0;
    // double y = 100000000.0;
    // corner case to check for begin of trajectory
    // double x = -0.5;
    // double y = -0.5;
    // corner case to check for end of trajectory
    // double x = 45.0;
    // double y = 0.0;

    cout << "Given point = " << x << " " << y << endl;

    // Task 2: convert the point in Cartesian coordinate to point in Frenet system
    pt_frenet = car1.cartesianToFrenet(x, y, pts_XY);
    cout << "frenet latitude = " << pt_frenet[0].first << " " << "frenet longitude = " << pt_frenet[0].second << endl;

    return 0;
}
