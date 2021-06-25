#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>

using std::cout;
using std::deque;
using std::vector;
using std::string;

class CostMap;

struct point { // or cell or coords
    CostMap* map;
    int x;
    int y;
};

// Maybe TODO: change combine column/row editing functions to be more intuitive/efficient
// TODO: look at MAAV repos, figure out integration

// class which stores a map of travel costs at each point and finds the optimal path between two points using the A* algorithm
class CostMap {
public:
    // ----- MAP -----
    // Constructor
    CostMap(int h, int w, point p, double m = 1) : height(h), width(w), pos(p), min(m) {
        if (min <= 0) {
            cout << "error: min cost value must be positive\n";
            exit(1);
        }
        cell_costs.resize(width);
        astar_data.resize(width);
        for (int i = 0; i < width; i++) {
	    cell_costs[i].resize(height, min);
            astar_data[i].resize(height);
        }
    }
    // Functions
    bool in_bounds(point p); // whether a point is in the map
    void set_cell_cost(point p, double cost); // set the cost of a single cell
    double get_cell_cost(point p); // get the cost of a single cell
    void add_columns(int n); // add n > 0 columns on the end of map or -n > 0 columns on the beginning of map
    void del_columns(int n); // delete n > 0 columns from the end of map or -n > 0 columns from the beginning of map
    void add_rows(int n); // add n > 0 rows on the end of map or -n > 0 rows on the beginning of map
    void del_rows(int n); // delete n > 0 rows from the end of map or -n > 0 rows from the beginning of map
    // Variables
    point pos;
    double min;
    int width;
    int height;

    // ----- A* -----
    // Functions
    double get_path_cost(point p); // cumulative cost of the minimum path to point p
    int heuristic(point p1, point p2); // heuristic cost of travel between two points (Manhattan distance)
    deque<point> find_path(point g); // find the optimal path to a goal g using the A* algorithm
    void print_cell_cost_map(); // print the movement cost of each cell
    void print_path_cost_map(); // print the cumulative cost of the minimum path to each cell evaluated so far
    void print_search_map(); // print evaluation status of each cell
    void print_path(); // print the coordinates of the cells the path runs through
    // Variables
    deque<point> path;
    point goal;

private:
    // ----- MAP -----
    // Variables
    deque<deque<double>> cell_costs;

    // ----- A* -----
    // Structs
    struct CellAstarData { // or [Cell]AstarInfo or CellPath[Data/Info]
        point prev = {0, 0, 0};
        double path_cost = std::numeric_limits<double>::max();
        bool added = 0;
        int visited = 0; // only for record keeping
    };
    // Variables
    deque<deque<CellAstarData>> astar_data;
    vector<point> border;
    point cur_pt;
    bool updated_since_astar = false;
    // Functions
    void reset_astar(); // prepare for next astar path search
    void update_neighbors(); // update attributes of neighboring cells (based on current cell attributes)
    int output_search(point p); // search results: 0 = untouched, 1 = added to border (evaluating cost), 2 = visited (cost evaluated), 3 = path (minimum cost)
    int output_path_cost(point p); // cumulative cost of the minimum path to point p, but replace max double values with 0
};

// determine which point has the lower cost according to the A* algorithm
class cheaper {
public:
    bool operator() (point p1, point p2) {
        if (p1.map != p2.map) {
            cout << "error: two points from different path searches compared\n";
            exit(1);
        }
        double p1_total = p1.map->get_path_cost(p1) + p1.map->heuristic(p1, p1.map->goal);
        double p2_total = p2.map->get_path_cost(p2) + p2.map->heuristic(p2, p2.map->goal);
        return p1_total > p2_total;
    }
};

// ----- MAP -----

// whether a point is in the map
bool CostMap::in_bounds(point p) {
    return 0 <= p.x && p.x < width && 0 <= p.y && p.y < height;
}

// set the cost of a single cell
void CostMap::set_cell_cost(point p, double cost) {
    if (cost <= 0) {
        cout << "error: cost must be positive\n";
        exit(1);
    }
    else if (cost < min) {
        cout << "warning: cost less than heuristic minimum; solution not guaranteed to be optimal\n";
    }
    updated_since_astar = true;
    cell_costs[p.x][p.y] = cost;
}

// get the cost of a single cell
double CostMap::get_cell_cost(point p) {
    return cell_costs[p.x][p.y];
}

// add n > 0 columns on the end of map or -n > 0 columns on the beginning of map
void CostMap::add_columns(int n) {
    updated_since_astar = true;
    if (n > 0) {
        cell_costs.insert(cell_costs.end(), n, deque<double>(height, min));
        width += n;
    }
    else {
        cell_costs.insert(cell_costs.begin(), -n, deque<double>(height, min));
        width -= n;
    }
    astar_data.resize(width);
}

// delete n > 0 columns from the end of map or -n > 0 columns from the beginning of map
void CostMap::del_columns(int n) {
    updated_since_astar = true;
    if (n > 0) {
        cell_costs.erase(cell_costs.end() - n, cell_costs.end());
        width -= n;
    }
    else {
        cell_costs.erase(cell_costs.begin(), cell_costs.begin() - n);
        width += n;
    }
    astar_data.resize(width);
}

// add n > 0 rows on the end of map or -n > 0 rows on the beginning of map
void CostMap::add_rows(int n) {
    updated_since_astar = true;
    if (n > 0) {
        for (int i = 0; i < width; i++) {
            cell_costs[i].insert(cell_costs[i].end(), n, min);
        }
        height += n;
    }
    else {
        for (int i = 0; i < width; i++) {
            cell_costs[i].insert(cell_costs[i].begin(), -n, min);
        }
        height -= n;
    }
    for (int i = 0; i < width; i++) {
        astar_data[i].resize(height);
    }
}

// delete n > 0 rows from the end of map or -n > 0 rows from the beginning of map
void CostMap::del_rows(int n) {
    updated_since_astar = true;
    if (n > 0) {
        for (int i = 0; i < width; i++) {
            cell_costs[i].erase(cell_costs[i].end() - n, cell_costs[i].end());
        }
        height -= n;
    }
    else {
        for (int i = 0; i < width; i++) {
            cell_costs[i].erase(cell_costs[i].begin(), cell_costs[i].begin() - n);
        }
        height += n;
    }
    for (int i = 0; i < width; i++) {
        astar_data[i].resize(height);
    }
}

// ----- A* -----

// cumulative cost of the minimum path to point p
double CostMap::get_path_cost(point p) {
    return astar_data[p.x][p.y].path_cost;
}

// heuristic cost of travel between two points (Manhattan distance)
int CostMap::heuristic(point p1, point p2) {
    return abs(p1.x - p2.x) + abs(p1.y - p2.y); // Manhattan distance
}

// prepare for next astar path search
void CostMap::reset_astar() {
    for (int i = 0; i < width; i++) {
	    for (int j = 0; j < height; j++) {
	        astar_data[i][j] = CellAstarData();
	    }
    }
    border.clear();
    path.clear();
}

// update attributes of neighboring cells (based on current cell attributes)
void CostMap::update_neighbors() {
    vector<point> sides{4};
    sides[0] = {this, cur_pt.x, cur_pt.y + 1};
    sides[1] = {this, cur_pt.x, cur_pt.y - 1};
    sides[2] = {this, cur_pt.x + 1, cur_pt.y};
    sides[3] = {this, cur_pt.x - 1, cur_pt.y};
    double new_cost;
    for (point side : sides) {
        if (!in_bounds(side)) continue;
        // update cost if new is less than existing
        new_cost = astar_data[cur_pt.x][cur_pt.y].path_cost + cell_costs[side.x][side.y];
        if (new_cost < astar_data[side.x][side.y].path_cost) {
            astar_data[side.x][side.y].path_cost = new_cost;
            astar_data[side.x][side.y].prev = cur_pt;
            // push to border if not added already
            if (!astar_data[side.x][side.y].added) {
                border.push_back({this, side.x, side.y});
                std::push_heap(border.begin(), border.end(), cheaper());
                astar_data[side.x][side.y].added = true;
            }
        }
    }
}

// find the optimal path to a goal g using the A* algorithm
deque<point> CostMap::find_path(point g) {
    goal = g;
    // if map hasn't changed since last run, results will be the same; otherwise, reset and start over
    if (!updated_since_astar) return path;
    else reset_astar();
    // set first border cell to starting point
    astar_data[pos.x][pos.y].path_cost = 0;
    astar_data[pos.x][pos.y].visited = true;
    astar_data[pos.x][pos.y].added = true;
    border.push_back(pos);
    // almost last spot before A* uses cost map (i.e. last spot when A* is guaranteed to be relying on up-to-date data)
    updated_since_astar = false;
    // expand border until goal is reached
    while (border[0].x != goal.x || border[0].y != goal.y) {
        cur_pt = border[0]; // go to point with the lowest cost
        astar_data[cur_pt.x][cur_pt.y].visited = true;
        std::pop_heap(border.begin(), border.end(), cheaper());
        border.pop_back(); // remove current point from border
        astar_data[cur_pt.x][cur_pt.y].added = false;
        update_neighbors(); // update costs and add to border as needed
        std::make_heap(border.begin(), border.end(), cheaper()); // update border in case element costs updated
        print_search_map();
        print_path_cost_map();
    }
    // reconstruct path from end to beginning
    cur_pt = goal;
    astar_data[cur_pt.x][cur_pt.y].visited = true;
    while (cur_pt.x != pos.x || cur_pt.y != pos.y) {
        path.push_front(cur_pt);
        cur_pt = astar_data[cur_pt.x][cur_pt.y].prev;
    }
    path.push_front(cur_pt);
    print_path();
    print_search_map();
    return path;
}

// search results: 0 = untouched, 1 = added to border (evaluating cost), 2 = visited (cost evaluated), 3 = path (minimum cost)
int CostMap::output_search(point p) {
    if (astar_data[p.x][p.y].visited == 2) return 3;
    return astar_data[p.x][p.y].added ? astar_data[p.x][p.y].added : astar_data[p.x][p.y].visited * 2;
}

// cumulative cost of the minimum path to point p, but replace max double values with 0
int CostMap::output_path_cost(point p) {
    return astar_data[p.x][p.y].path_cost == std::numeric_limits<double>::max() ? 0 : astar_data[p.x][p.y].path_cost;
}

// print the movement cost of each cell
void CostMap::print_cell_cost_map() {
    cout << "\ncell cost map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++)
            cout << cell_costs[j][i] << '\t';
        cout << '\n';
    }
}

// print the cumulative cost of the minimum path to each cell evaluated so far
void CostMap::print_path_cost_map() {
    cout << "\npath cost map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++)
            cout << output_path_cost({ this, j, i }) << '\t';
        cout << '\n';
    }
}

// print evaluation status of each cell
void CostMap::print_search_map() {
    cout << "\nsearch map:\n";
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++)
            cout << output_search({ this, j, i }) << ' ';
        cout << '\n';
    }
}

// print the coordinates of the cells the path runs through
void CostMap::print_path() {
    cout << "\npath coordinates:\n";
    for (point pt : path) {
        astar_data[pt.x][pt.y].visited = 2;
        cout << pt.x << ',' << pt.y << " (cost = " << astar_data[pt.x][pt.y].path_cost << ")\n";
    }
}
