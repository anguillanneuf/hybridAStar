#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include "hybrid_breadth_first.h"

using namespace std;


int X = 1;
int _ = 0;

int NUM_THETA_CELLS = 90;
double SPEED = 1.45;
double LENGTH = 0.5;

// how is this an integer matrix?
vector<vector<int> > MAZE = {
        {_, X, X, _, _, _, _, _, _, _, X, X, _, _, _, _,},
        {_, X, X, _, _, _, _, _, _, X, X, _, _, _, _, _,},
        {_, X, X, _, _, _, _, _, X, X, _, _, _, _, _, _,},
        {_, X, X, _, _, _, _, X, X, _, _, _, X, X, X, _,},
        {_, X, X, _, _, _, X, X, _, _, _, X, X, X, _, _,},
        {_, X, X, _, _, X, X, _, _, _, X, X, X, _, _, _,},
        {_, X, X, _, X, X, _, _, _, X, X, X, _, _, _, _,},
        {_, X, X, X, X, _, _, _, X, X, X, _, _, _, _, _,},
        {_, X, X, X, _, _, _, X, X, X, _, _, _, _, _, _,},
        {_, X, X, _, _, _, X, X, X, _, _, X, X, X, X, X,},
        {_, X, _, _, _, X, X, X, _, _, X, X, X, X, X, X,},
        {_, _, _, _, X, X, X, _, _, X, X, X, X, X, X, X,},
        {_, _, _, X, X, X, _, _, X, X, X, X, X, X, X, X,},
        {_, _, X, X, X, _, _, X, X, X, X, X, X, X, X, X,},
        {_, X, X, X, _, _, _, _, _, _, _, _, _, _, _, _,},
        {X, X, X, _, _, _, _, _, _, _, _, _, _, _, _, _,},
};


vector<vector<int> > GRID = MAZE;

vector<double> START = {0.0, 0.0, 0.0};
vector<int> GOAL = {(int) GRID.size() - 1, (int) GRID[0].size() - 1};

//vector<vector<int>> makeBlankGrid(int size) {
//    vector<vector<int>> grid;
//    vector<int> row;
//    for (int cols = 0; cols < size; cols++) {
//        row.clear();
//        for (int rows = 0; rows < size; rows++) {
//            row.push_back(0);
//        }
//        grid.push_back(row);
//    }
//    return grid;
//}

int main() {

//    vector< vector<int> > GRID = makeBlankGrid(16);

    cout << "Finding path through grid:" << endl;

    // TODO:: Create an Empty Maze and try testing the number of expansions with it

    for (int i = 0; i < GRID.size(); i++) {
        cout << GRID[i][0];
        for (int j = 1; j < GRID[0].size(); j++) {
            cout << "," << GRID[i][j];
        }
        cout << endl;
    }

    HBF hbf = HBF();

    cout << "Check my heuristic function. " << endl;

//    vector<vector<double> > heuristic = hbf.heuristic_basic(GRID, START, GOAL);
    vector<vector<double> > heuristic = hbf.heuristic_nhh_euclidean(GRID, GOAL);
//    vector<vector<double> > heuristic = hbf.heuristic_nhh_manhattan(GRID, GOAL);

    for (int i = 0; i < heuristic.size(); i++) {
        cout << heuristic[i][0];
        for (int j = 1; j < heuristic[0].size(); j++) {
            cout << "," << heuristic[i][j];
        }
        cout << endl;
    }

    HBF::maze_path get_path = hbf.search(GRID, START, GOAL);

    vector<HBF::maze_s> show_path = hbf.reconstruct_path(get_path.came_from, START, get_path.final);

    cout << "show path from start to finish" << endl;
    for (int i = show_path.size() - 1; i >= 0; i--) {

        HBF::maze_s step = show_path[i];
        cout << "##### step " << step.g << " #####" << endl;
        cout << "x " << step.x << endl;
        cout << "y " << step.y << endl;
        cout << "theta " << step.theta << endl;

    }

    return 0;
}