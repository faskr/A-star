#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>

using std::cout;
using std::deque;
using std::vector;
using std::string;

class AstarPath2D;

struct point {
    AstarPath2D* A;
    int x;
    int y;
};

// Class for finding the A* path between two points given a map of location-specific travel costs
class AstarPath2D {
public:
    // Constructor
    AstarPath2D(int h, int w, point p, point g) : height(h), width(w), pos(p), goal(g) {
        info.resize(width);
        for (int i = 0; i < width; i++) {
            info[i].resize(height, {{0, 0, 0}, -1, std::numeric_limits<double>::max(), 0, 0});
        }
    }
    // Functions
    void print_path();
    void print_cost_map();
    void print_search_map();
    point get_pos() { return pos; }
    point get_goal() { return goal; }
    deque<point> get_path() { return path; }
    deque<point> find_path();
    double cell_cost(point p) { return info[p.x][p.y].cell_cost; }
    double path_cost(point p) { return info[p.x][p.y].path_cost; }
    // set the cost of a single cell
    void set_cell_cost(point p, double cost) {
        if (info[p.x][p.y].cell_cost > -1) { // find_path shouldn't run twice
            cout << "error: cannot set block value twice\n";
            exit(1);
        }
        info[p.x][p.y].cell_cost = cost;
    }
    // set the cost of all cells at once
    void set_map_cost(const vector<vector<double>> &costs) {
        // map sizes must be equal, and find_path shouldn't be run twice
        if (costs.size() != width || costs[0].size() != height || map_cost_set) {
            cout << "error: invalid cost map\n";
	    exit(1);
	}
	map_cost_set = true;
        for (int i = 0; i < width; i++) {
	    for (int j = 0; j < height; j++) {
                info[i][j].cell_cost = costs[i][j];
	    }
	}
    }
    // whether a point is in the map
    bool in_bounds(point p) {
        return 0 <= p.x && p.x < width && 0 <= p.y && p.y < height;
    }
    // heuristic cost of travel between two points
    int heuristic(point p1, point p2) {
        return abs(p1.x - p2.x) + abs(p1.y - p2.y); // Manhattan distance
    }

private:
    // Structs
    struct CellInfo {
        point prev;
        double cell_cost;
        double path_cost;
        int added;
        int visited;
    };
    // Variables
    vector<vector<CellInfo>> info;
    vector<point> border;
    deque<point> path;
    point pos;
    point goal;
    point cur_pt;
    int width;
    int height;
    bool map_cost_set = false;
    // Functions
    // search results: 0 = untouched, 1 = added to border (evaluating cost), 2 = visited (cost determined), 3 = path (minimum cost)
    int output_search(point p) {
        if (info[p.x][p.y].visited == 2) return 3;
        return info[p.x][p.y].visited ? info[p.x][p.y].visited * 2 : info[p.x][p.y].added;
    }
    // path cost of point
    int output_cost(point p) {
        return info[p.x][p.y].path_cost == std::numeric_limits<double>::max() ? -1 : info[p.x][p.y].path_cost;
    }
    // update cells next to current cell
    void update_neighbors();
};

// Determine which point has a lower cost according to the A* algorithm
class cheaper {
public:
    bool operator() (point p1, point p2) {
        if (p1.A != p2.A) {
            cout << "error: two points from different path searches compared\n";
            exit(1);
        }
        double p1_total = p1.A->path_cost(p1) + p1.A->heuristic(p1, p1.A->get_goal());
        double p2_total = p2.A->path_cost(p2) + p2.A->heuristic(p2, p2.A->get_goal());
        return p1_total > p2_total;
    }
};

void AstarPath2D::print_cost_map() {
    cout << "\ncost map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++)
            cout << output_cost({ this, j, i }) << '\t';
        cout << '\n';
    }
}

void AstarPath2D::print_search_map() {
    cout << "\nsearch map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++)
            cout << output_search({ this, j, i }) << ' ';
        cout << '\n';
    }
}

void AstarPath2D::print_path() {
    cout << "\npath coordinates:\n";
    for (point pt : path) {
        info[pt.x][pt.y].visited = 2;
        cout << pt.x << ',' << pt.y << " (cost = " << info[pt.x][pt.y].path_cost << ")\n";
    }
}

// Update attributes of neighboring cells (based on current cell attributes)
void AstarPath2D::update_neighbors() {
    vector<point> sides{4};
    sides[0] = {this, cur_pt.x, cur_pt.y + 1};
    sides[1] = {this, cur_pt.x, cur_pt.y - 1};
    sides[2] = {this, cur_pt.x + 1, cur_pt.y};
    sides[3] = {this, cur_pt.x - 1, cur_pt.y};
    double new_cost;
    bool new_cost_less;
    for (point side : sides) {
        if (!in_bounds(side)) continue;
        // update cost if cell is not visited (i.e. not the cheapest border cell) and new cost is less than existing
        new_cost = info[cur_pt.x][cur_pt.y].path_cost + info[side.x][side.y].cell_cost;
        new_cost_less = new_cost < info[side.x][side.y].path_cost;
        if (!info[side.x][side.y].visited && new_cost_less) { // TODO: is checking for visited necessary or even efficient?
            info[side.x][side.y].path_cost = new_cost;
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

// Find path according to A* algorithm
deque<point> AstarPath2D::find_path() {
    // set first border cell to starting point
    info[pos.x][pos.y].path_cost = 0; //map[pos.x][pos.y];
    info[pos.x][pos.y].visited = true;
    info[pos.x][pos.y].added = true;
    border.push_back(pos);
    // expand border until goal is reached
    while (!info[goal.x][goal.y].visited) {
        cur_pt = border[0]; // go to point with the lowest cost
        info[cur_pt.x][cur_pt.y].visited = true;
        std::pop_heap(border.begin(), border.end(), cheaper());
        border.pop_back(); // remove current point from border
        update_neighbors(); // update costs and add to border as needed
        std::make_heap(border.begin(), border.end(), cheaper()); // update border in case element costs updated
        print_search_map();
        print_cost_map();
    }
    // reconstruct path from end to beginning
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
