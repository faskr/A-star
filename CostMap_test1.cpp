#include <string>
#include <fstream>
#include "CostMap.h"

int main() {
    srand(time(0));
    int height = rand() % 10 + 10;
    int width = rand() % 10 + 10;
    point pos = {nullptr, rand() % width, rand() % height};
    point goal = {nullptr, rand() % width, rand() % height};
    CostMap A(height, width, pos);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            if (rand() % 8 == 0) {
                A.set_cell_cost({ &A, j, i }, rand() % 5 + 1);
            }
        }
    }
    A.find_path(goal);
    return 0;
}