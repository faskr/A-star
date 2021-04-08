#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <string>
#include <algorithm>

using std::cout;
using std::endl;
using std::deque;
using std::vector;
using std::string;

struct point {
    int x;
    int y;
};

class Astar {
public:
    struct Info {
        double map;
        double cost;
        point prev;
        int added;
        int visited;
    };

    static vector<vector<Info>> info;
    static point goal;

    Astar() {}

    Astar(int h, int w, point p) : height(h), width(w), pos(p) {
        info.resize(width);
        for (int i = 0; i < width; i++) {
            info[i].resize(height, {0, std::numeric_limits<double>::max(), 0, 0, 0});
        }
    }

    bool in_bounds(int x, int y, int width, int height) {
        return 0 <= x && x < width && 0 <= y && y < height;
    }

    int heuristic(point p1, point p2) {
        return abs(p1.x - p2.x) + abs(p1.y - p2.y); // Manhattan distance
    }

    int output_path(int i, int j) {
        if (info[j][i].visited == 2) return 3;
        return output_search(i, j);
    }

    int output_search(int i, int j) {
        return info[j][i].visited ? info[j][i].visited * 2 : info[j][i].added;
    }

    int output_cost(int i, int j) {
        return info[j][i].cost == std::numeric_limits<double>::max() ? -1 : info[j][i].cost;
    }

    void print_map(string type, int (Astar::*output)(int, int), char delim);
    void print_coords();

    deque<point> find_path();

private:
    vector<point> border;
    deque<point> path;
    point pos;
    point cur_pt;
    int width;
    int height;

    void update_neighbors();
};

vector<vector<Astar::Info>> Astar::info;
point Astar::goal;

class cheaper : public Astar {
public:
    bool operator() (point p1, point p2) {
        return info[p1.x][p1.y].cost + heuristic(p1, goal) > info[p2.x][p2.y].cost + heuristic(p2, goal);
    }
};

void Astar::print_map(string type, int (Astar::*output)(int, int), char delim) {
    cout << '\n' << type << " map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            cout << (this->*output)(i, j) << delim;
        }
        cout << '\n';
    }
}

void Astar::print_coords() {
    cout << "\npath coordinates:\n";
    for (point pt : path) {
        info[pt.x][pt.y].visited = 2;
        cout << pt.x << ',' << pt.y << " (cost = " << info[pt.x][pt.y].cost << ")\n";
    }
}

void Astar::update_neighbors() {
    vector<point> sides{4};
    sides[0] = {cur_pt.x, cur_pt.y + 1};
    sides[1] = {cur_pt.x, cur_pt.y - 1};
    sides[2] = {cur_pt.x + 1, cur_pt.y};
    sides[3] = {cur_pt.x - 1, cur_pt.y};
    double new_cost;
    bool new_cost_less;
    for (point side : sides) {
        // update cost if block is not visited and new cost is less than existing
        if (!in_bounds(side.x, side.y, width, height)) continue;
        new_cost = info[cur_pt.x][cur_pt.y].cost + info[side.x][side.y].map;
        new_cost_less = new_cost < info[side.x][side.y].cost;
        if (!info[side.x][side.y].visited && new_cost_less) {
            info[side.x][side.y].cost = new_cost;
            info[side.x][side.y].prev = cur_pt;
        }
        // push to border if not added already
        if (!info[side.x][side.y].added) {
            border.push_back({side.x, side.y});
            std::push_heap(border.begin(), border.end(), cheaper());
            info[side.x][side.y].added = true;
        }
    }
}

deque<point> Astar::find_path() {
    info[pos.x][pos.y].cost = 0; //map[pos.x][pos.y];
    info[pos.x][pos.y].visited = true;
    info[pos.x][pos.y].added = true;
    border.push_back(pos);
    while (!info[goal.x][goal.y].visited) {
        cur_pt = border[0]; // go to point with the lowest cost
        info[cur_pt.x][cur_pt.y].visited = true;
        std::pop_heap(border.begin(), border.end(), cheaper());
        border.pop_back(); // remove current point from border
        update_neighbors(); // update costs and add to border as needed
        std::make_heap(border.begin(), border.end(), cheaper()); // update in case element costs updated
        print_map("search", &Astar::output_search, ' ');
        print_map("cost", &Astar::output_cost, '\t');
    }
    cur_pt = goal;
    while (cur_pt.x != pos.x || cur_pt.y != pos.y) {
        path.push_front(cur_pt);
        cur_pt = info[cur_pt.x][cur_pt.y].prev;
    }
    path.push_front(cur_pt);
    print_coords();
    print_map("path", &Astar::output_path, ' ');
    return path;
}

int import_and_run(string filename) {
    std::ifstream ifs(filename);
    int height, width;
    point pos, goal;
    ifs >> height >> width >> pos.x >> pos.y >> goal.x >> goal.y;
    Astar A(height, width, pos);
    if (!A.in_bounds(pos.x, pos.y, width, height)) return 1;
    if (!A.in_bounds(goal.x, goal.y, width, height)) return 1;
    A.goal = goal;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            ifs >> A.info[j][i].map;
        }
    }
    A.find_path();
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc < 2) return 0;
    return import_and_run(argv[1]);
}