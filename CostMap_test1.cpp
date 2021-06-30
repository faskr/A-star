#include <string>
#include <fstream>
#include "CostMap.h"

int main() {
    int height = rand() % 10 + 1;
    int width = rand() % 10 + 1;
    point pos = {nullptr, rand() % width, rand() % height};
    point goal = {nullptr, rand() % width, rand() % height};
    CostMap A(height, width, pos);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            A.set_cell_cost({ &A, j, i }, rand() % 10 + 1);
        }
    }
    A.find_path(goal);
    return 0;
}