#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <deque>
#include <math.h>
#include <algorithm>
#include <string>
#include <sstream>

using std::cout;
using std::endl;
using std::deque;
using std::vector;
using std::string;

struct point {
    int x;
    int y;
    //int z;
};

void travel(int cur, int next) {
    uint32_t x = 1;
    while (x != 0) {
        x++;
    }
}

bool in_bounds(int x, int y, int width, int height) {
    return 0 <= x && x < width && 0 <= y && y < height;
}

int main(int argc, char* argv[]) {
    if (argc < 2) return 0;
    std::ifstream ifs(argv[1]);
    int height;
    int width;
    point pos;
    point goal;
    ifs >> height;
    ifs >> width;
    ifs >> pos.x;
    ifs >> pos.y;
    ifs >> goal.x;
    ifs >> goal.y;
    vector<vector<double>> map(width);
    vector<vector<point>> prev(width);
    vector<vector<int>> reached(width);
    for (int i = 0; i < width; i++) {
        map[i].resize(height);
        prev[i].resize(height);
        reached[i].resize(height);
        for (int j = 0; j < height; j++) {
            ifs >> map[i][j];
        }
    }
    deque<point> border;
    reached[pos.x][pos.y] = true;
    border.push_back(pos);
    point cur_pt;
    while (!reached[goal.x][goal.y]) {
        cur_pt = border.front();
        border.pop_front();
        if (in_bounds(cur_pt.x, cur_pt.y + 1, width, height) && !reached[cur_pt.x][cur_pt.y + 1]) {
            reached[cur_pt.x][cur_pt.y + 1] = true;
            border.push_back({cur_pt.x, cur_pt.y + 1});
            prev[cur_pt.x][cur_pt.y + 1] = cur_pt;
        }
        if (in_bounds(cur_pt.x, cur_pt.y - 1, width, height) && !reached[cur_pt.x][cur_pt.y - 1]) {
            reached[cur_pt.x][cur_pt.y - 1] = true;
            border.push_back({cur_pt.x, cur_pt.y - 1});
            prev[cur_pt.x][cur_pt.y - 1] = cur_pt;
        }
        if (in_bounds(cur_pt.x + 1, cur_pt.y, width, height) && !reached[cur_pt.x + 1][cur_pt.y]) {
            reached[cur_pt.x + 1][cur_pt.y] = true;
            border.push_back({cur_pt.x + 1, cur_pt.y});
            prev[cur_pt.x + 1][cur_pt.y] = cur_pt;
        }
        if (in_bounds(cur_pt.x - 1, cur_pt.y, width, height) && !reached[cur_pt.x - 1][cur_pt.y]) {
            reached[cur_pt.x - 1][cur_pt.y] = true;
            border.push_back({cur_pt.x - 1, cur_pt.y});
            prev[cur_pt.x - 1][cur_pt.y] = cur_pt;
        }
    }
    cur_pt = goal;
    deque<point> path;
    while (cur_pt.x != pos.x || cur_pt.y != pos.y) {
        path.push_front(cur_pt);
        cur_pt = prev[cur_pt.x][cur_pt.y];
    }
    path.push_front(cur_pt);
    // print
    cout << "path coordinates:\n";
    for (point pt : path) {
        reached[pt.x][pt.y] = 2;
        cout << pt.x << ',' << pt.y << '\n';
    }
    cout << "\npath map:\n";
    for (int i = height - 1; i >= 0; i--) {
        for (int j = 0; j < width; j++) {
            cout << reached[j][i] << ' ';
        }
        cout << '\n';
    }
}