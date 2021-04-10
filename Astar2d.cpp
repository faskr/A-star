#include <iostream>
#include <fstream>
#include <vector>
#include <queue>
#include <string>
#include <algorithm>

using std::cout;
using std::deque;
using std::vector;
using std::string;

class Astar;

struct point {
    Astar* A;
    int x;
    int y;
};

// TODO: put everything related to class in header file
// TODO: consider renaming Astar class or instances to path (or AstarPath) or journey or a synonym
// TODO: write set_map function to set the entire map at once; consider making map a pointer to a map instead of stored values
// TODO: documentation via comments
class Astar {
public:
    Astar(int h, int w, point p, point g) : height(h), width(w), pos(p), goal(g) {
        info.resize(width);
        for (int i = 0; i < width; i++) {
            info[i].resize(height, {{0, 0, 0}, -1, std::numeric_limits<double>::max(), 0, 0});
        }
    }

    void print_path();
    void print_cost_map();
    void print_search_map();
    point get_pos() { return pos; }
    point get_goal() { return goal; }
    deque<point> get_path() { return path; }
    deque<point> find_path();
    double block_cost(point p) { return info[p.x][p.y].map; }
    double move_cost(point p) { return info[p.x][p.y].cost; }

    void set_block(point p, double cost) {
        if (info[p.x][p.y].map > -1) { // find_path shouldn't run twice
            cout << "error: cannot set block value twice\n";
            exit(1);
        }
        info[p.x][p.y].map = cost;
    }

    bool in_bounds(point p) {
        return 0 <= p.x && p.x < width && 0 <= p.y && p.y < height;
    }

    int heuristic(point p1, point p2) {
        return abs(p1.x - p2.x) + abs(p1.y - p2.y); // Manhattan distance
    }

private:
    struct Info {
        point prev;
        double map; // TODO: change var name
        double cost;
        int added;
        int visited;
    };

    vector<vector<Info>> info;
    vector<point> border;
    deque<point> path;
    point pos;
    point goal;
    point cur_pt;
    int width;
    int height;

    int output_search(point p) {
        if (info[p.x][p.y].visited == 2) return 3;
        return info[p.x][p.y].visited ? info[p.x][p.y].visited * 2 : info[p.x][p.y].added;
    }

    int output_cost(point p) {
        return info[p.x][p.y].cost == std::numeric_limits<double>::max() ? -1 : info[p.x][p.y].cost;
    }

    void update_neighbors();
};

class cheaper {
public:
    bool operator() (point p1, point p2) {
        if (p1.A != p2.A) {
            cout << "error: two points from different path searches compared\n";
            exit(1);
        }
        double p1_total = p1.A->move_cost(p1) + p1.A->heuristic(p1, p1.A->get_goal());
        double p2_total = p2.A->move_cost(p2) + p2.A->heuristic(p2, p2.A->get_goal());
        return p1_total > p2_total;
    }
};

void Astar::print_cost_map() {
    cout << "\ncost map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++)
            cout << output_cost({ this, j, i }) << '\t';
        cout << '\n';
    }
}

void Astar::print_search_map() {
    cout << "\nsearch map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++)
            cout << output_search({ this, j, i }) << ' ';
        cout << '\n';
    }
}

void Astar::print_path() {
    cout << "\npath coordinates:\n";
    for (point pt : path) {
        info[pt.x][pt.y].visited = 2;
        cout << pt.x << ',' << pt.y << " (cost = " << info[pt.x][pt.y].cost << ")\n";
    }
}

void Astar::update_neighbors() {
    vector<point> sides{4};
    sides[0] = {this, cur_pt.x, cur_pt.y + 1};
    sides[1] = {this, cur_pt.x, cur_pt.y - 1};
    sides[2] = {this, cur_pt.x + 1, cur_pt.y};
    sides[3] = {this, cur_pt.x - 1, cur_pt.y};
    double new_cost;
    bool new_cost_less;
    for (point side : sides) {
        // update cost if block is not visited and new cost is less than existing
        if (!in_bounds(side)) continue;
        new_cost = info[cur_pt.x][cur_pt.y].cost + info[side.x][side.y].map;
        new_cost_less = new_cost < info[side.x][side.y].cost;
        if (!info[side.x][side.y].visited && new_cost_less) {
            info[side.x][side.y].cost = new_cost;
            info[side.x][side.y].prev = cur_pt;
        }
        // push to border if not added already
        if (!info[side.x][side.y].added) {
            border.push_back({this, side.x, side.y});
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
        print_search_map();
        print_cost_map();
    }
    cur_pt = goal;
    while (cur_pt.x != pos.x || cur_pt.y != pos.y) {
        path.push_front(cur_pt);
        cur_pt = info[cur_pt.x][cur_pt.y].prev;
    }
    path.push_front(cur_pt);
    print_path();
    print_search_map();
    return path;
}

int import_and_run(string filename) {
    std::ifstream ifs(filename);
    int height, width;
    point pos, goal;
    ifs >> height >> width >> pos.x >> pos.y >> goal.x >> goal.y;
    Astar A(height, width, pos, goal);
    if (!A.in_bounds(pos)) return 1;
    if (!A.in_bounds(goal)) return 1;
    double cost;
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            ifs >> cost;
            A.set_block({ &A, j, i }, cost);
        }
    }
    A.find_path();
    return 0;
}

int main(int argc, char* argv[]) {
    if (argc < 2) return 0;
    return import_and_run(argv[1]);
}