#include <iostream>
#include <stdlib.h>
#include "trajectory.hpp"
// main
int main(){

    // open file to read the coordinates before plotting them
    ifstream file("input.txt");
    // Car object
    Car car1;

    vector<int> pts_X;
    vector<int> pts_Y;
    vector<pair<double, double>> pts_XY;
    vector<pair<double, double>> pt_frenet;

    double X = 0;
    double Y = 0;

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

    // print the XY coordinate pairs
    cout << pts_XY << endl;

    // plot the reference trajectory from the XY coordinates
    car1.plotRefTraj(pts_XY);
    // mouse click event that triggers cartesianToFrenet method of object car1
    double x = 0.0;
    double y = 0.0;
    // trigger cartesianToFrenet method of object car1
    pt_frenet = car1.cartesianToFrenet(x, y, pts_XY);
    // display frenet point in the plot/screen
    cout << pt_frenet << endl;

    return 0;
}
