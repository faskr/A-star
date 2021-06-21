#include "AstarPath2D.h"

int import_and_run(string filename) {
    std::ifstream ifs(filename);
    int height, width;
    point pos, goal;
    ifs >> height >> width >> pos.x >> pos.y >> goal.x >> goal.y;
    AstarPath2D A(height, width, pos, goal);
    if (!A.in_bounds(pos)) return 1;
    if (!A.in_bounds(goal)) return 1;
    double cost;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            ifs >> cost;
            A.set_cell_cost({ &A, j, i }, cost);
        }
    }
    A.find_path();
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc < 2) return 0;
    return import_and_run(argv[1]);
}
