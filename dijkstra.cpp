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
};

struct Info {
    double map;
    double cost;
    point prev;
    int place;
    int reached;
};

vector<vector<Info>> info;

class cheaper {
public:
    bool operator() (point p1, point p2) {
        if (info[p1.x][p1.y].cost != info[p2.x][p2.y].cost)
            return info[p1.x][p1.y].cost > info[p2.x][p2.y].cost;
        return info[p1.x][p1.y].place > info[p2.x][p2.y].place;
    }
};

bool in_bounds(int x, int y, int width, int height) {
    return 0 <= x && x < width && 0 <= y && y < height;
}

bool update_cost_up(point cur_pt, int width, int height) {
    return in_bounds(cur_pt.x, cur_pt.y + 1, width, height) &&
        (!info[cur_pt.x][cur_pt.y + 1].reached || info[cur_pt.x][cur_pt.y].cost +
        info[cur_pt.x][cur_pt.y + 1].map < info[cur_pt.x][cur_pt.y + 1].cost);
}

bool update_cost_down(point cur_pt, int width, int height) {
    return in_bounds(cur_pt.x, cur_pt.y - 1, width, height) &&
        (!info[cur_pt.x][cur_pt.y - 1].reached || info[cur_pt.x][cur_pt.y].cost +
        info[cur_pt.x][cur_pt.y - 1].map < info[cur_pt.x][cur_pt.y - 1].cost);
}

bool update_cost_right(point cur_pt, int width, int height) {
    return in_bounds(cur_pt.x + 1, cur_pt.y, width, height) &&
        (!info[cur_pt.x + 1][cur_pt.y].reached || info[cur_pt.x][cur_pt.y].cost +
        info[cur_pt.x + 1][cur_pt.y].map < info[cur_pt.x + 1][cur_pt.y].cost);
}

bool update_cost_left(point cur_pt, int width, int height) {
    return in_bounds(cur_pt.x - 1, cur_pt.y, width, height) &&
        (!info[cur_pt.x - 1][cur_pt.y].reached || info[cur_pt.x][cur_pt.y].cost +
        info[cur_pt.x - 1][cur_pt.y].map < info[cur_pt.x - 1][cur_pt.y].cost);
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
    info.resize(width);
    for (int i = 0; i < width; i++) {
        info[i].resize(height, {0, std::numeric_limits<double>::max(), 0, 0, 0});
    }
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            ifs >> info[j][i].map;
        }
    }
    priority_queue<point, vector<point>, cheaper> border;
    info[pos.x][pos.y].cost = 0; //map[pos.x][pos.y];
    info[pos.x][pos.y].reached = true;
    border.push(pos);
    point cur_pt;
    int cur_place = 0;
    while (!info[goal.x][goal.y].reached) {
        cur_pt = border.top();
        border.pop();
        if (update_cost_up(cur_pt, width, height)) {
            info[cur_pt.x][cur_pt.y + 1].cost = info[cur_pt.x][cur_pt.y].cost + info[cur_pt.x][cur_pt.y + 1].map;
            info[cur_pt.x][cur_pt.y + 1].prev = cur_pt;
            info[cur_pt.x][cur_pt.y + 1].place = ++cur_place;
            info[cur_pt.x][cur_pt.y + 1].reached = true;
            border.push({cur_pt.x, cur_pt.y + 1});
        }
        if (update_cost_down(cur_pt, width, height)) {
            info[cur_pt.x][cur_pt.y - 1].cost = info[cur_pt.x][cur_pt.y].cost + info[cur_pt.x][cur_pt.y - 1].map;
            info[cur_pt.x][cur_pt.y - 1].prev = cur_pt;
            info[cur_pt.x][cur_pt.y - 1].place = ++cur_place;
            info[cur_pt.x][cur_pt.y - 1].reached = true;
            border.push({cur_pt.x, cur_pt.y - 1});
        }
        if (update_cost_right(cur_pt, width, height)) {
            info[cur_pt.x + 1][cur_pt.y].cost = info[cur_pt.x][cur_pt.y].cost + info[cur_pt.x + 1][cur_pt.y].map;
            info[cur_pt.x + 1][cur_pt.y].prev = cur_pt;
            info[cur_pt.x + 1][cur_pt.y].place = ++cur_place;
            info[cur_pt.x + 1][cur_pt.y].reached = true;
            border.push({cur_pt.x + 1, cur_pt.y});
        }
        if (update_cost_left(cur_pt, width, height)) {
            info[cur_pt.x - 1][cur_pt.y].cost = info[cur_pt.x][cur_pt.y].cost + info[cur_pt.x - 1][cur_pt.y].map;
            info[cur_pt.x - 1][cur_pt.y].prev = cur_pt;
            info[cur_pt.x - 1][cur_pt.y].place = ++cur_place;
            info[cur_pt.x - 1][cur_pt.y].reached = true;
            border.push({cur_pt.x - 1, cur_pt.y});
        }
    }
    cur_pt = goal;
    deque<point> path;
    while (cur_pt.x != pos.x || cur_pt.y != pos.y) {
        path.push_front(cur_pt);
        cur_pt = info[cur_pt.x][cur_pt.y].prev;
    }
    path.push_front(cur_pt);
    // print
    cout << "path coordinates:\n";
    for (point pt : path) {
        info[pt.x][pt.y].reached = 2;
        cout << pt.x << ',' << pt.y << " (cost = " << info[pt.x][pt.y].cost << ")\n";
    }
    cout << "\npath map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            cout << info[j][i].reached << ' ';
        }
        cout << '\n';
    }
}