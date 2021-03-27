#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <math.h>
#include <algorithm>
#include <string>
#include <sstream>

using std::cout;
using std::endl;
using std::priority_queue;
using std::deque;
using std::vector;
using std::string;

struct point {
    int x;
    int y;
    //int z;
    int dist;
};

class closer {
public:
    bool operator() (point p1, point p2) {
        return p1.dist > p2.dist;
    }
};

bool in_bounds(int x, int y, int width, int height) {
    return 0 <= x && x < width && 0 <= y && y < height;
}

int heuristic(int p1x, int p1y, point p2) {
    return abs(p1x - p2.x) + abs(p1y - p2.y); // Manhattan distance
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
    }
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            ifs >> map[j][i];
        }
    }
    priority_queue<point, vector<point>, closer> border;
    reached[pos.x][pos.y] = true;
    border.push(pos);
    point cur_pt;
    while (!reached[goal.x][goal.y]) {
        cur_pt = border.top();
        border.pop();
        if (in_bounds(cur_pt.x, cur_pt.y + 1, width, height) && !reached[cur_pt.x][cur_pt.y + 1]) {
            reached[cur_pt.x][cur_pt.y + 1] = true;
            border.push({cur_pt.x, cur_pt.y + 1, heuristic(cur_pt.x, cur_pt.y + 1, goal)});
            prev[cur_pt.x][cur_pt.y + 1] = cur_pt;
        }
        if (in_bounds(cur_pt.x, cur_pt.y - 1, width, height) && !reached[cur_pt.x][cur_pt.y - 1]) {
            reached[cur_pt.x][cur_pt.y - 1] = true;
            border.push({cur_pt.x, cur_pt.y - 1, heuristic(cur_pt.x, cur_pt.y - 1, goal)});
            prev[cur_pt.x][cur_pt.y - 1] = cur_pt;
        }
        if (in_bounds(cur_pt.x + 1, cur_pt.y, width, height) && !reached[cur_pt.x + 1][cur_pt.y]) {
            reached[cur_pt.x + 1][cur_pt.y] = true;
            border.push({cur_pt.x + 1, cur_pt.y, heuristic(cur_pt.x + 1, cur_pt.y, goal)});
            prev[cur_pt.x + 1][cur_pt.y] = cur_pt;
        }
        if (in_bounds(cur_pt.x - 1, cur_pt.y, width, height) && !reached[cur_pt.x - 1][cur_pt.y]) {
            reached[cur_pt.x - 1][cur_pt.y] = true;
            border.push({cur_pt.x - 1, cur_pt.y, heuristic(cur_pt.x - 1, cur_pt.y, goal)});
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
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            cout << reached[j][i] << ' ';
        }
        cout << '\n';
    }
}