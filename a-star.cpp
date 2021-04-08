#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <math.h>
#include <algorithm>
#include <string>
#include <sstream>
#include <functional>
#include <cassert>

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

class Astar {
public:
    struct Info {
        double map;
        double cost;
        point prev;
        int place;
        int visited;
        int added;
    };

    static vector<vector<Info>> info;
    //vector<point> path;
    point pos;
    point goal;
    int width;
    int height;

    //priority_queue<point, vector<point>, std::function<bool(point, point)>> border(cheaper);
    vector<point> border;
    point cur_pt;
    //vector<point> sides;
    int cur_place = 0;
    deque<point> path;

    bool in_bounds(int x, int y, int width, int height) {
        return 0 <= x && x < width && 0 <= y && y < height;
    }

    int heuristic(point p1, point p2) {
        return abs(p1.x - p2.x) + abs(p1.y - p2.y); // Manhattan distance
    }

    void update_neighbors();
    void find_path();

    int output_path(int i, int j);
    int output_search(int i, int j);
    int output_cost(int i, int j);
    void print_map(string type, int (Astar::*output)(int, int), char delim);
    void print_coords();

    //void print_map_with_border();
    //void print_cost_map();
};

vector<vector<Astar::Info>> Astar::info;

// TODO: consider making friend class instead
class cheaper : public Astar {
public:
    bool operator() (point p1, point p2) {
        double priority1 = info[p1.x][p1.y].cost + heuristic(p1, goal);
        double priority2 = info[p2.x][p2.y].cost + heuristic(p2, goal);
        //if (priority1 != priority2)
            return priority1 > priority2;
        //return info[p1.x][p1.y].place > info[p2.x][p2.y].place;
    }
};

void Astar::update_neighbors() {
    vector<point> sides{4};
    sides[0] = {cur_pt.x, cur_pt.y + 1};
    sides[1] = {cur_pt.x, cur_pt.y - 1};
    sides[2] = {cur_pt.x + 1, cur_pt.y};
    sides[3] = {cur_pt.x - 1, cur_pt.y};
    bool update;
    bool add;
    for (point side : sides) {
        // update cost if block is not visited and new cost is less than existing
        update = in_bounds(side.x, side.y, width, height) &&
            !info[side.x][side.y].visited && info[cur_pt.x][cur_pt.y].cost +
            info[side.x][side.y].map < info[side.x][side.y].cost;
        if (update) {
            info[side.x][side.y].cost = info[cur_pt.x][cur_pt.y].cost + info[side.x][side.y].map;
            info[side.x][side.y].prev = cur_pt;
            info[side.x][side.y].place = ++cur_place;
        }
        // push to border if not added already
        add = in_bounds(side.x, side.y, width, height) &&
            !info[side.x][side.y].added;
        if (add) {
            border.push_back({side.x, side.y});
            std::push_heap(border.begin(), border.end(), cheaper());
            info[side.x][side.y].added = true;
        }
    }
}

void Astar::print_coords() {
    cout << "\npath coordinates:\n";
    for (point pt : path) {
        info[pt.x][pt.y].visited = 2;
        cout << pt.x << ',' << pt.y << " (cost = " << info[pt.x][pt.y].cost << ")\n";
    }
}

/*
void Astar::print_map_with_border() {
    cout << "\nsearch map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            cout << (info[j][i].visited ? info[j][i].visited * 2 : info[j][i].added) << ' ';
        }
        cout << '\n';
    }
}

void Astar::print_cost_map() {
    cout << "\ncost map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            cout << (info[j][i].cost > 99 ? -1 : info[j][i].cost) << '\t';
        }
        cout << '\n';
    }
}
*/

void Astar::find_path() {
    info[pos.x][pos.y].cost = 0; //map[pos.x][pos.y];
    info[pos.x][pos.y].visited = true;
    info[pos.x][pos.y].added = true;
    border.push_back(pos);
    while (!info[goal.x][goal.y].visited) {
        // go to point with the lowest cost
        assert(!border.empty());
        cur_pt = border[0];
        // remove current point from border
        std::pop_heap(border.begin(), border.end(), cheaper());
        border.pop_back();
        // update costs and add to border as needed
        update_neighbors();
        // current point has now been visited
        info[cur_pt.x][cur_pt.y].visited = true;
        // update border queue in case costs of elements were updated
        std::make_heap(border.begin(), border.end(), cheaper());
        // print
        print_map("search", &Astar::output_search, ' ');
        print_map("cost", &Astar::output_cost, '\t');
        //print_map_with_border();
        //print_cost_map();
    }
    cur_pt = goal;
    while (cur_pt.x != pos.x || cur_pt.y != pos.y) {
        path.push_front(cur_pt);
        cur_pt = info[cur_pt.x][cur_pt.y].prev;
    }
    path.push_front(cur_pt);
    print_coords();
    print_map("path", &Astar::output_path, ' ');
}

int Astar::output_path(int i, int j) {
    return info[j][i].visited;
}

int Astar::output_search(int i, int j) {
    return info[j][i].visited ? info[j][i].visited * 2 : info[j][i].added;
}

int Astar::output_cost(int i, int j) {
    return info[j][i].cost > 99 ? -1 : info[j][i].cost;
}

void Astar::print_map(string type, int (Astar::*output)(int, int), char delim) {
    cout << '\n' << type << " map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            cout << (this->*output)(i, j) << delim;
        }
        cout << '\n';
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) return 0;
    std::ifstream ifs(argv[1]);
    Astar A;
    ifs >> A.height;
    ifs >> A.width;
    ifs >> A.pos.x;
    ifs >> A.pos.y;
    ifs >> A.goal.x;
    ifs >> A.goal.y;
    A.info.resize(A.width);
    for (int i = 0; i < A.width; i++) {
        A.info[i].resize(A.height, {0, std::numeric_limits<double>::max(), 0, 0, 0});
    }
    for (int i = 0; i < A.height; i++) {
        for (int j = 0; j < A.width; j++) {
            ifs >> A.info[j][i].map;
        }
    }
    A.find_path();
}