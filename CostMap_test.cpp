#include <string>
#include <fstream>
#include "CostMap.h"

// Read in map information, initialize A-star class, and run path finder
int import_and_run(string filename) {
    std::ifstream ifs(filename);
    int height, width;
    point pos, goal;
    ifs >> height >> width >> pos.x >> pos.y >> goal.x >> goal.y;
    CostMap A(height - 1, width, pos);
    //if (!A.in_bounds(pos)) return 1;
    //if (!A.in_bounds(goal)) return 1;
    double cost;
    for (int i = 0; i < height - 1; i++) {
        for (int j = 0; j < width; j++) {
            ifs >> cost;
            A.set_cell_cost({ &A, j, i }, cost);
        }
    }
    A.add_rows(1);
    for (int j = 0; j < width; j++) {
        ifs >> cost;
        A.set_cell_cost({ &A, j, height - 1 }, cost);
    }
    A.find_path(goal);
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc < 2) return 0;
    return import_and_run(argv[1]);
}
