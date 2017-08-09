//
// Created by Tianzi Harrison on 8/8/17.
//

#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "hybrid_breadth_first.h"

using namespace std;

/**
 * Initializes HBF
 */
HBF::HBF() {

}

HBF::~HBF() {}


int HBF::theta_to_stack_number(double theta) {
    /*
    Takes an angle (in radians) and returns which "stack" in the 3D configuration space
    this angle corresponds to. Angles near 0 go in the lower stacks while angles near
    2 * pi go in the higher stacks.
    */

    double new_theta = fmod((theta + 2 * M_PI), (2 * M_PI));
    int stack_number = (int) (round(new_theta * NUM_THETA_CELLS / (2 * M_PI))) % NUM_THETA_CELLS;
    return stack_number;
}


int HBF::idx(double float_num) {
    /*
    Returns the index into the grid for continuous position. So if x is 3.621, then this
    would return 3 to indicate that 3.621 corresponds to array index 3.
    */

    return int(floor(float_num));
}


vector<HBF::maze_s> HBF::expand(HBF::maze_s state) {
    int g = state.g;
    double x = state.x;
    double y = state.y;
    double theta = state.theta;

    int g2 = g + 1;
    vector<HBF::maze_s> next_states;
    for (double delta_i = -35; delta_i < 40; delta_i += 5) {

        double delta = M_PI / 180.0 * delta_i;
        double omega = SPEED / LENGTH * tan(delta);
        double theta2 = theta + omega;
        if (theta2 > 0) {
            theta2 += 2 * M_PI;
        }
        double x2 = x + SPEED * cos(theta2);
        double y2 = y + SPEED * sin(theta2);
        HBF::maze_s state2;
        state2.g = g2;
        state2.x = x2;
        state2.y = y2;
        state2.theta = theta2;
        next_states.push_back(state2);

    }
    return next_states;
}

vector<HBF::maze_s>
HBF::reconstruct_path(vector<vector<vector<HBF::maze_s> > > came_from, vector<double> start, HBF::maze_s final) {

    vector<maze_s> path = {final};

    int stack = theta_to_stack_number(final.theta);

    maze_s current = came_from[stack][idx(final.x)][idx(final.y)];

    stack = theta_to_stack_number(current.theta);

    double x = current.x;
    double y = current.y;
    while (x != start[0] && y != start[1]) {
        path.push_back(current);
        current = came_from[stack][idx(x)][idx(y)];
        x = current.x;
        y = current.y;
        stack = theta_to_stack_number(current.theta);
    }

    return path;

}

HBF::maze_path HBF::search(vector<vector<int> > grid, vector<double> start, vector<int> goal) {
    /*
    Working Implementation of breadth first search. Does NOT use a heuristic
    and as a result this is pretty inefficient. Try modifying this algorithm
    into hybrid A* by adding heuristics appropriately.
    */

    vector<vector<vector<maze_s> > > closed(NUM_THETA_CELLS,
                                            vector<vector<maze_s>>(grid[0].size(), vector<maze_s>(grid.size())));
    vector<vector<vector<int> > > closed_value(NUM_THETA_CELLS,
                                               vector<vector<int>>(grid[0].size(), vector<int>(grid.size())));
    vector<vector<vector<maze_s> > > came_from(NUM_THETA_CELLS,
                                               vector<vector<maze_s>>(grid[0].size(), vector<maze_s>(grid.size())));
    double theta = start[2];
    int stack = theta_to_stack_number(theta);
    int g = 0;

    maze_s state;
    state.g = g;
    state.x = start[0];
    state.y = start[1];

    closed[stack][idx(state.x)][idx(state.y)] = state;
    closed_value[stack][idx(state.x)][idx(state.y)] = 1;
    came_from[stack][idx(state.x)][idx(state.y)] = state;
    int total_closed = 1;
    vector<maze_s> opened = {state};
    bool finished = false;

    // add my heuristic function
    vector<vector<double> > heuristic = heuristic_nhh_euclidean(grid, goal);
//    vector<vector<double> > heuristic = heuristic_nhh_manhattan(grid, goal);


    while (!opened.empty()) {

        maze_s next = opened[0]; //grab first element
        opened.erase(opened.begin()); //pop first element

        int x = next.x;
        int y = next.y;

        if (idx(x) == goal[0] && idx(y) == goal[1]) {
            cout << "found path to goal in " << total_closed << " expansions" << endl;
            maze_path path;
            path.closed = closed;
            path.came_from = came_from;
            path.final = next;
            return path;

        }
        vector<maze_s> next_state = expand(next);

        double minValue = 999;

        for (int i = 0; i < next_state.size(); i++) {
            int g2 = next_state[i].g + heuristic[x][y];
            double x2 = next_state[i].x;
            double y2 = next_state[i].y;
            double theta2 = next_state[i].theta;

            if ((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size())) {
                //invalid cell
                continue;
            }
            int stack2 = theta_to_stack_number(theta2);

            if (closed_value[stack2][idx(x2)][idx(y2)] == 0 && grid[idx(x2)][idx(y2)] == 0) {

                maze_s state2;
                state2.g = g2;
                state2.x = x2;
                state2.y = y2;
                state2.theta = theta2;

                opened.push_back(state2);

                closed[stack2][idx(x2)][idx(y2)] = next_state[i];
                closed_value[stack2][idx(x2)][idx(y2)] = 1;
                came_from[stack2][idx(x2)][idx(y2)] = next;
                total_closed += 1;
                if (g2 < minValue)
                    minValue = g2;
            }


        }

    }
    cout << "no valid path." << endl;
    HBF::maze_path path;
    path.closed = closed;
    path.came_from = came_from;
    path.final = state;
    return path;

}

vector<vector<int> > HBF::heuristic_basic(vector<vector<int> > grid, vector<double> start, vector<int> goal){
    /*
    Create the heuristic function for A star searches on a grid, given start position and goal position.
    */

    vector<vector<int> > heuristic = grid;
    vector<vector<int> > closed = grid;

    // give obstacles a very large value
    for (int i = 0; i < heuristic.size(); i ++){
        for (int j = 0; j < heuristic[0].size(); j ++){
            if (heuristic[i][j] == 1){
                heuristic[i][j] = 999;
            }
        }
    }

    vector<vector<int>> opened = {{int(start[0]),int(start[1])}};

    heuristic[int(start[0])][int(start[1])] = 1;

    while (!opened.empty() ) {

        vector<int> next = opened[0]; //grab first element
        opened.erase(opened.begin()); //pop first element

        int x = next[0];
        int y = next[1];

        if (x == goal[0] && y == goal[1]) {
            break;
        }

        vector<vector<int>> next_state;
        next_state.push_back({x-1,y});
        next_state.push_back({x,y+1});
        next_state.push_back({x+1,y});
        next_state.push_back({x,y-1});

        int g = heuristic[x][y];

        for (int i = 0; i < next_state.size(); i++) {

            int x2 = next_state[i][0];
            int y2 = next_state[i][1];
            int temp = 0;

            if ((x2 < 0 || x2 >= grid.size()) || (y2 < 0 || y2 >= grid[0].size())) {
                //invalid cell
                continue;
            }

            if (heuristic[idx(x2)][idx(y2)] == 0 & closed[x2][y2] == 0) {
                temp += 1;
                opened.push_back(next_state[i]);
                heuristic[idx(x2)][idx(y2)] = g + max(temp, 1);
                closed[x2][y2] = 1;
            }

        }

    }

    // any remaining unreached cell takes on 999
    for (int i = 0; i < heuristic.size(); i ++){
        for (int j = 0; j < heuristic[0].size(); j ++){
            if (heuristic[i][j] == 0){
                heuristic[i][j] = 999;
            }
        }
    }

    return heuristic;

}

vector<vector<double> > HBF::heuristic_nhh_euclidean(vector<vector<int> > grid, vector<int> goal){
    /*
    Create the heuristic function for non-holonomic heuristic, which does not consider obstacles.
    A simple implementation.
    */

    vector<vector<double> > heuristic(grid.size(), vector<double>(grid[0].size(), 0));

    // give obstacles a very large value
    for (int i = 0; i < grid.size(); i ++){
        for (int j = 0; j < grid[0].size(); j ++){
            heuristic[i][j] = sqrt(pow(abs(i-goal[1]),2)+pow(abs(j-goal[0]),2));
        }
    }

    return heuristic;

}

vector<vector<double> > HBF::heuristic_nhh_manhattan(vector<vector<int> > grid, vector<int> goal){
    /*
    Create the heuristic function for non-holonomic heuristic, which does not consider obstacles.
    A simple implementation.
    */

    vector<vector<double> > heuristic(grid.size(), vector<double>(grid[0].size(), 0));

    // give obstacles a very large value
    for (int i = 0; i < grid.size(); i ++){
        for (int j = 0; j < grid[0].size(); j ++){
            heuristic[i][j] = abs(i-goal[1])+abs(j-goal[0]);
        }
    }

    return heuristic;

}
