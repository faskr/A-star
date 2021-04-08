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

    /*
    void update_sides();
    vector<bool> check_update_costs();
    void update_costs();
    void push_border();
    */

    void update_neighbors();

    /*
    bool check_update_cost_up();
    bool check_update_cost_down();
    bool check_update_cost_right();
    bool check_update_cost_left();

    void update_cost_up();
    void update_cost_down();
    void update_cost_right();
    void update_cost_left();

    void push_border_up();
    void push_border_down();
    void push_border_right();
    void push_border_left();
    */

    void find_path();
    void print_coords();
    void print_map();
    void print_map_with_border();
    void print_cost_map();
};

vector<vector<Astar::Info>> Astar::info;

// TODO: consider making friend class instead
class cheaper : public Astar {
public:
    bool operator() (point p1, point p2) {
        if (info[p1.x][p1.y].cost + heuristic(p1, goal) != info[p2.x][p2.y].cost + heuristic(p2, goal))
            return info[p1.x][p1.y].cost + heuristic(p1, goal) > info[p2.x][p2.y].cost + heuristic(p2, goal);
        return info[p1.x][p1.y].place > info[p2.x][p2.y].place;
    }
};

/*
void Astar::update_sides() {
    sides[0] = {cur_pt.x, cur_pt.y + 1};
    sides[1] = {cur_pt.x, cur_pt.y - 1};
    sides[2] = {cur_pt.x + 1, cur_pt.y};
    sides[3] = {cur_pt.x - 1, cur_pt.y};
}

vector<bool> Astar::check_update_costs() {
    update_sides();
    vector<bool> update{4};
    for (int i = 0; i < 4; i++) {
        update[i] = in_bounds(sides[i].x, sides[i].y, width, height) &&
            (!info[sides[i].x][sides[i].y].visited || info[cur_pt.x][cur_pt.y].cost +
            info[sides[i].x][sides[i].y].map < info[sides[i].x][sides[i].y].cost);
    }
    return update;
}

void Astar::update_costs() {
    vector<bool> update = check_update_costs();
    for (int i = 0; i < 4; i++) {
        if (update[i]) {
            info[sides[i].x][sides[i].y].cost = info[cur_pt.x][cur_pt.y].cost + info[sides[i].x][sides[i].y].map;
            info[sides[i].x][sides[i].y].prev = cur_pt;
            info[sides[i].x][sides[i].y].place = ++cur_place;
        }
    }
}

void Astar::push_border() {
    for (point side : sides) {
        if (!info[side.x][side.y].added) {
            border.push_back({side.x, side.y});
            std::push_heap(border.begin(), border.end(), cheaper());
            info[side.x][side.y].added = true;
        }
    }
}
*/

void Astar::update_neighbors() {
    vector<point> sides{4};
    sides[0] = {cur_pt.x, cur_pt.y + 1};
    sides[1] = {cur_pt.x, cur_pt.y - 1};
    sides[2] = {cur_pt.x + 1, cur_pt.y};
    sides[3] = {cur_pt.x - 1, cur_pt.y};
    bool update;
    bool add;
    for (point side : sides) {
        update = in_bounds(side.x, side.y, width, height) &&
            !info[side.x][side.y].visited && info[cur_pt.x][cur_pt.y].cost +
            info[side.x][side.y].map < info[side.x][side.y].cost;
        if (update) {
            info[side.x][side.y].cost = info[cur_pt.x][cur_pt.y].cost + info[side.x][side.y].map;
            info[side.x][side.y].prev = cur_pt;
            info[side.x][side.y].place = ++cur_place;
        }
        add = in_bounds(side.x, side.y, width, height) &&
            !info[side.x][side.y].added;
        if (add) {
            border.push_back({side.x, side.y});
            std::push_heap(border.begin(), border.end(), cheaper());
            info[side.x][side.y].added = true;
        }
    }
}

/*
bool Astar::check_update_cost_up() {
    return in_bounds(cur_pt.x, cur_pt.y + 1, width, height) &&
        (!info[cur_pt.x][cur_pt.y + 1].visited || info[cur_pt.x][cur_pt.y].cost +
        info[cur_pt.x][cur_pt.y + 1].map < info[cur_pt.x][cur_pt.y + 1].cost);
}

bool Astar::check_update_cost_down() {
    return in_bounds(cur_pt.x, cur_pt.y - 1, width, height) &&
        (!info[cur_pt.x][cur_pt.y - 1].visited || info[cur_pt.x][cur_pt.y].cost +
        info[cur_pt.x][cur_pt.y - 1].map < info[cur_pt.x][cur_pt.y - 1].cost);
}

bool Astar::check_update_cost_right() {
    return in_bounds(cur_pt.x + 1, cur_pt.y, width, height) &&
        (!info[cur_pt.x + 1][cur_pt.y].visited || info[cur_pt.x][cur_pt.y].cost +
        info[cur_pt.x + 1][cur_pt.y].map < info[cur_pt.x + 1][cur_pt.y].cost);
}

bool Astar::check_update_cost_left() {
    return in_bounds(cur_pt.x - 1, cur_pt.y, width, height) &&
        (!info[cur_pt.x - 1][cur_pt.y].visited || info[cur_pt.x][cur_pt.y].cost +
        info[cur_pt.x - 1][cur_pt.y].map < info[cur_pt.x - 1][cur_pt.y].cost);
}

void Astar::update_cost_up() {
    if (check_update_cost_up()) {
        info[cur_pt.x][cur_pt.y + 1].cost = info[cur_pt.x][cur_pt.y].cost + info[cur_pt.x][cur_pt.y + 1].map;
        info[cur_pt.x][cur_pt.y + 1].prev = cur_pt;
        info[cur_pt.x][cur_pt.y + 1].place = ++cur_place;
    }
}

void Astar::update_cost_down() {
    if (check_update_cost_down()) {
        info[cur_pt.x][cur_pt.y - 1].cost = info[cur_pt.x][cur_pt.y].cost + info[cur_pt.x][cur_pt.y - 1].map;
        info[cur_pt.x][cur_pt.y - 1].prev = cur_pt;
        info[cur_pt.x][cur_pt.y - 1].place = ++cur_place;
    }
}

void Astar::update_cost_right() {
    if (check_update_cost_right()) {
        info[cur_pt.x + 1][cur_pt.y].cost = info[cur_pt.x][cur_pt.y].cost + info[cur_pt.x + 1][cur_pt.y].map;
        info[cur_pt.x + 1][cur_pt.y].prev = cur_pt;
        info[cur_pt.x + 1][cur_pt.y].place = ++cur_place;
    }
}

void Astar::update_cost_left() {
    if (check_update_cost_left()) {
        info[cur_pt.x - 1][cur_pt.y].cost = info[cur_pt.x][cur_pt.y].cost + info[cur_pt.x - 1][cur_pt.y].map;
        info[cur_pt.x - 1][cur_pt.y].prev = cur_pt;
        info[cur_pt.x - 1][cur_pt.y].place = ++cur_place;
    }
}

void Astar::push_border_up() {
    if (!info[cur_pt.x][cur_pt.y + 1].added) {
        border.push_back({cur_pt.x, cur_pt.y + 1});
        std::push_heap(border.begin(), border.end(), cheaper());
        info[cur_pt.x][cur_pt.y + 1].added = true;
    }
}

void Astar::push_border_down() {
    if (!info[cur_pt.x][cur_pt.y - 1].added) {
        border.push_back({cur_pt.x, cur_pt.y - 1});
        std::push_heap(border.begin(), border.end(), cheaper());
        info[cur_pt.x][cur_pt.y - 1].added = true;
    }
}

void Astar::push_border_right() {
    if (!info[cur_pt.x + 1][cur_pt.y].added) {
        border.push_back({cur_pt.x + 1, cur_pt.y});
        std::push_heap(border.begin(), border.end(), cheaper());
        info[cur_pt.x + 1][cur_pt.y].added = true;
    }
}

void Astar::push_border_left() {
    if (!info[cur_pt.x - 1][cur_pt.y].added) {
        border.push_back({cur_pt.x - 1, cur_pt.y});
        std::push_heap(border.begin(), border.end(), cheaper());
        info[cur_pt.x - 1][cur_pt.y].added = true;
    }
}
*/

void Astar::print_coords() {
    cout << "\npath coordinates:\n";
    for (point pt : path) {
        info[pt.x][pt.y].visited = 2;
        cout << pt.x << ',' << pt.y << " (cost = " << info[pt.x][pt.y].cost << ")\n";
    }
}

void Astar::print_map() {
    cout << "\npath map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            cout << info[j][i].visited << ' ';
        }
        cout << '\n';
    }
}

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

void Astar::find_path() {
    info[pos.x][pos.y].cost = 0; //map[pos.x][pos.y];
    info[pos.x][pos.y].visited = true;
    info[pos.x][pos.y].added = true;
    border.push_back(pos);
    while (!info[goal.x][goal.y].visited) { // TODO: maybe change to / add condition that border is empty, i.e. no more points in map to inspect
        // go to point with the lowest cost
        assert(!border.empty());
        cur_pt = border[0];
        // remove current point from border
        std::pop_heap(border.begin(), border.end(), cheaper());
        border.pop_back();
        // update costs if less than existing
        //update_costs();
        // push to border if not added already
        //push_border();
        update_neighbors();
        // current point has now been visited
        info[cur_pt.x][cur_pt.y].visited = true;
        // update border queue in case costs of elements were updated
        std::make_heap(border.begin(), border.end(), cheaper());
        print_map_with_border();
        print_cost_map();
    }
    cur_pt = goal;
    while (cur_pt.x != pos.x || cur_pt.y != pos.y) {
        path.push_front(cur_pt);
        cur_pt = info[cur_pt.x][cur_pt.y].prev;
    }
    path.push_front(cur_pt);
    print_coords();
    print_map();
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