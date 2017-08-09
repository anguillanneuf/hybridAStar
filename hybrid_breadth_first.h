//
// Created by Tianzi Harrison on 8/8/17.
//

#ifndef HYBRIDASTAR_HYBRID_BREADTH_FIRST_H
#define HYBRIDASTAR_HYBRID_BREADTH_FIRST_H

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>

using namespace std;

class HBF {
public:

    int NUM_THETA_CELLS = 90;
    double SPEED = 1.45;
    double LENGTH = 0.5;

    struct maze_s {

        double g;    // iteration
        double x;
        double y;
        double theta;
    };

    struct maze_path {

        vector<vector<vector<maze_s> > > closed;
        vector<vector<vector<maze_s> > > came_from;
        maze_s final;

    };


    /**
      * Constructor
      */
    HBF();

    /**
     * Destructor
     */
    virtual ~HBF();


    int theta_to_stack_number(double theta);

    int idx(double float_num);

    vector<maze_s> expand(maze_s state);

    maze_path search(vector<vector<int> > grid, vector<double> start, vector<int> goal);

    vector<maze_s>
    reconstruct_path(vector<vector<vector<maze_s> > > came_from, vector<double> start, HBF::maze_s final);

    vector<vector<int>> heuristic_basic(vector<vector<int>> grid, vector<double> start, vector<int> goal);
    vector<vector<double>> heuristic_nhh_euclidean(vector<vector<int>> grid, vector<int> goal);
    vector<vector<double>> heuristic_nhh_manhattan(vector<vector<int>> grid, vector<int> goal);
};

class comparator {
public:
    bool operator() (HBF::maze_s a, HBF::maze_s b) {
        return a.g < b.g;
    }
};


#endif //HYBRIDASTAR_HYBRID_BREADTH_FIRST_H
