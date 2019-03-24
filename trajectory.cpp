// C++ program to print reference trajectory for given set of coordinates

#include <iostream>
#include <fstream>
#include <vector>
#include "gnuplot-iostream.h"

using namespace std;

// Class for the ego car
class Car{
    public:

    int plotRefTraj();

};

int Car::plotRefTraj(){
    return 0;
}

// int Car::getFrenetPoints(){
//
// }

int main(){
    Car Car1;
    Gnuplot gp;

    // open file to read the coordinates before plotting them
    ifstream file_obj;
    file_obj.open("input.txt", ios::in);

    Car obj;
    file_obj.read((char*) &obj, sizeof(obj));

    vector<int> pts_X;
    vector<int> pts_Y;
    std::vector<std::pair<double, double>> pts_XY;
    int X = 0;
    int Y = 0;

    // scan each line in the file to update the points to the respective array
    while(!file_obj.eof()){
        // update X array
        file_obj>>X>>Y;
        pts_X.push_back(X);
        // update Y array
        pts_Y.push_back(Y);
        pts_XY.push_back(make_pair(X,Y));

    }


    // pass reference points to get the reference trajectory of the car
    // Car1.plotRefTraj();
    gp << "plot" << gp.file1d(pts_XY) << "Reference Trajectory" << endl;
    // pass random points to check them in the Frenet coordinate system
    // Car1.getFrenetPoints();


    return 0;
}
