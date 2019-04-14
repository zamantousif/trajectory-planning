//#include "trajectory.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <limits>
#include <string>
#include <boost/lexical_cast.hpp>
#include "gnuplot-iostream.h"
using boost::lexical_cast;
using std::string;
using namespace std;
// use full precision for double
typedef numeric_limits<double> dbl;

// define constants
const double LARGEVAL = 1000000.0;

// function to compute distance between two points in cartesian plane
double getDistance(double x1, double y1, double x2, double y2){
    return sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
}

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

    // ensure that the chosen waypoint is such that the vector drawn from it to the given point (x,y) makes an acute angle


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
    double vx, vy, sx, sy, x1, y1, x2, y2;
    double scaling_fac;
    double frenet_long = 0.0;
    vector <pair<double, double>> frenet_vec;
    int waypoint_num;
    bool leftX = false;
    bool leftY = false;

    waypoint_num = findWaypoint(x, y, v);

    // nearest waypoint to given point (x,y)
    x1 = v[waypoint_num].first;
    y1 = v[waypoint_num].second;
    // waypoint that comes ahead of the waypoint nearest to (x,y)
    x2 = v[waypoint_num+1].first;
    y2 = v[waypoint_num+1].second;

    cout << "x2 = " << x2 << " " << "y2 = " << y2 << endl;
    cout << "x1 = " << x1 << " " << "y1 = " << y1 << endl;
    // compute x and y components of vector V between (x,y) and the nearest waypoint (x1,y1) on the reference trajectory
    vx = x - x1;
    vy = y - y1;
    cout << "vector V = " << vx << " " << vy << endl;
    // compute x and y components of vector S between nearest waypoint (x1,y1) and the next waypoint (x2,y2) both located on the reference trajectory
    sx = x2 - x1;
    sy = y2 - y1;
    cout << "vector S = " << sx << " " << sy << endl;
    // next step is to compute the projection of (x,y) on the reference trajectory
    // vector s lies on the reference trajectory and therefore ks.(a-ks) = 0
    // k = some scaling factor for vector s
    // k = v.s/s.s where "." is the vector dot product
    scaling_fac = (vx*sx + vy*sy)/(sx*sx + sy*sy);
    double proj_x = scaling_fac*sx;
    double proj_y = scaling_fac*sy;
    cout << "projection_x = " << proj_x << " " << "projection_y = " << proj_y << endl;

    // latitude on the frenet coordinate system
    double frenet_lat = getDistance(x, y, proj_x, proj_y);

    // check if the given point (x,y) is on the left/right of the reference trajectory and update frenet latitude
    if(x >= proj_x)
        leftX = true;

    if(y >= proj_y)
        leftY = true;

    if(leftX && leftY){
        frenet_lat *= -1;
    }

    // if the point lies on the reference trajectory itself, update frenet latitude to 0
    if((proj_x == 0.0) && (proj_y == 0.0)){
        frenet_lat = 0;
    }

    // longitude on the frenet coordinate system
    // compute distance until the waypoint nearest to the given point (x,y)
    for(int i = 0; i < waypoint_num; ++i){
        frenet_long += getDistance(v[i].first, v[i].second, v[i+1].first, v[i+1].second);
    }
    // add up the distance of this waypoint to the projection of (x,y)
    frenet_long += getDistance(proj_x, proj_y, v[waypoint_num].first, v[waypoint_num].second);


    // pair frenet_lat and frenet_long and store the pair in vector
    frenet_vec.push_back(make_pair(frenet_lat, frenet_long));

    return frenet_vec;
}

// main
int main(){

    // open file to read the coordinates before plotting them
    ifstream file("input.txt");
    // Car object
    Car car1;
    vector<pair<double, double>> pts_XY;
    vector<pair<double, double>> pt_frenet;
    double X = 0.0;
    double Y = 0.0;

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

    // Task 1 : print the XY coordinate pairs
    cout << std::setprecision(numeric_limits<double>::max_digits10) << pts_XY << endl;
    // alternative way to print out the values with best precision match
    // cout << lexical_cast<string>(pts_XY) << endl;

    // Task 1 : plot the reference trajectory from the XY coordinates
    car1.plotRefTraj(pts_XY);

    // mouse click event that triggers cartesianToFrenet method of object car1
    // double x = 41.9993400574;
    // double y = -0.101188264787;
    double x = 2.5;
    double y = 2.5;
    

    cout << "Given point = " << x << " " << y << endl;
    // trigger cartesianToFrenet method of object car1
    pt_frenet = car1.cartesianToFrenet(x, y, pts_XY);
    // Task 2: convert the point in Cartesian coordinate to point in Frenet system
    cout << "frenet latitude = " << pt_frenet[0].first << " " << "frenet longitude = " << pt_frenet[0].second << endl;

    return 0;
}
