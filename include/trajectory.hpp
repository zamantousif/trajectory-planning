#pragma once
#include <vector>
#include <fstream>

using namespace std;

// define constants
const double LARGEVAL = 1000000.0;

double distance(double x1, double y1, double x2, double y2);

int findWaypoint(double x, double y, const vector<pair<double, double>>& v);

// C++ template to print the contents of the vector
template <typename d1, typename d2>
ostream& operator<<(ostream& os, const vector<d1, d2>& v)
{
    // iterate over the vector v to print each coordinate pair
    for(unsigned int i = 0; i < v.size(); ++i)
        os << "(" << v[i].first << ", " << v[i].second << ")";

    return os;
}

// Class for the ego car
class Car{
    public:

    int plotRefTraj(const vector<pair<double, double>>& v);
    vector<pair<double, double>> cartesianToFrenet(double x, double y, const vector<pair<double, double>>& v);
};
