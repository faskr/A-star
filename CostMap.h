#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>

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

// TODO: write function that takes in a path of cells and outputs a smoother path of waypoints
//	idea: put segment endpoints where direction changes and continues for more than one cell
// TODO: consider using Euclidean distance heuristic to solve the problem of L-shaped paths
// Maybe TODO: consider including diagonal neighbors (requires doubling cost on diagonals or using heuristic besides Manhattan)

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
    void reshape_top(int n); // add n > 0 or remove -n > 0 rows to/from the top side of the cost map
    void reshape_bottom(int n); // add n > 0 or remove -n > 0 rows to/from the bottom side of the cost map
    void reshape_right(int n); // add n > 0 or remove -n > 0 columns to/from the right side of the cost map
    void reshape_left(int n); // add n > 0 or remove -n > 0 columns to/from the left side of the cost map
    // Variables
    point pos;
    double min;
    int width;
    int height;

    // ----- A* -----
    // Functions
    double get_path_cost(point p); // cumulative cost of the minimum path to point p
    double heuristic(point p1, point p2); // heuristic cost of travel between two points (Euclidean distance)
    deque<point> find_path(point g); // find the optimal path to a goal g using the A* algorithm
    void print_cell_cost_map(); // print the movement cost of each cell
    void print_path_cost_map(); // print the cumulative cost of the minimum path to each cell evaluated so far
    void print_search_map(); // print evaluation status of each cell
    void print_path(); // print the coordinates of the cells the path runs through
    // Variables
    deque<point> path;
    deque<point> waypoints;
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
        int added = 0;
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
    void smooth_path();
    void smooth_path_v1();
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

// add n > 0 or remove -n > 0 rows to/from the top side of the cost map
void CostMap::reshape_top(int n) {
    updated_since_astar = true;
    if (n > 0)
        for (int i = 0; i < width; i++)
            cell_costs[i].insert(cell_costs[i].begin(), n, min);
    else if (n < 0)
        for (int i = 0; i < width; i++)
            cell_costs[i].erase(cell_costs[i].begin(), cell_costs[i].begin() - n);
    else return;
    height += n;
    for (int i = 0; i < width; i++)
        astar_data[i].resize(height);
}

// add n > 0 or remove -n > 0 rows to/from the bottom side of the cost map
void CostMap::reshape_bottom(int n) {
    updated_since_astar = true;
    if (n > 0)
        for (int i = 0; i < width; i++)
            cell_costs[i].insert(cell_costs[i].end(), n, min);
    else if (n < 0)
        for (int i = 0; i < width; i++)
            cell_costs[i].erase(cell_costs[i].end() + n, cell_costs[i].end());
    else return;
    height += n;
    for (int i = 0; i < width; i++)
        astar_data[i].resize(height);
}

// add n > 0 or remove -n > 0 columns to/from the left side of the cost map
void CostMap::reshape_left(int n) {
    updated_since_astar = true;
    if (n > 0)
        cell_costs.insert(cell_costs.begin(), n, deque<double>(height, min));
    else if (n < 0)
        cell_costs.erase(cell_costs.begin(), cell_costs.begin() - n);
    else return;
    width += n;
    astar_data.resize(width);
}

// add n > 0 or remove -n > 0 columns to/from the right side of the cost map
void CostMap::reshape_right(int n) {
    updated_since_astar = true;
    if (n > 0)
        cell_costs.insert(cell_costs.end(), n, deque<double>(height, min));
    else if (n < 0)
        cell_costs.erase(cell_costs.end() + n, cell_costs.end());
    else return;
    width += n;
    astar_data.resize(width);
}

// ----- A* -----

// cumulative cost of the minimum path to point p
double CostMap::get_path_cost(point p) {
    return astar_data[p.x][p.y].path_cost;
}

// heuristic cost of travel between two points (Euclidean distance)
double CostMap::heuristic(point p1, point p2) {
    //return abs(p1.x - p2.x) + abs(p1.y - p2.y); // Manhattan distance
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy); // Euclidean distance
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
    waypoints.clear();
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

/*
Path segment: a line segment that has endpoints on the path defined by the smoothing algorithm
Rule: All path segments must either have at least one component with length 1 or be made up of path segments which all have at least one component with length 1
Alt Rule: All path segments must either have at least one component in [-1, 1] or be made up of path segments which all have at least one component in [-1, 1]
Rule: Sequential path segments with the same slope must be combined into one path segment
Potential Rule: A path segment p must have at least one component = 0 if and only if the following hold:
    1. p already has at least one component = 0 when the path modifies its course in a new direction for > 1 cell
    2. The new direction of the path must have at least one component whose sign is opposite to the sign of the same component in the path segment before p
*/
void CostMap::smooth_path() {
    if (path.empty()) return;
    if (path.size() == 1) {
        waypoints.push_back(path[0]);
        return;
    }
    point prev_dir = {this, path[1].x - path[0].x, path[1].y - path[0].y};
    point cur_dir;
    point prev_slope = {this, 0, 0};
    point cur_slope = prev_dir;
    int wp_candidate = 0;
    for (int i = 2; i < path.size(); i++) {
        cur_dir.x = path[i].x - path[i-1].x;
        cur_dir.y = path[i].y - path[i-1].y;
        cur_slope.x += cur_dir.x;
        cur_slope.y += cur_dir.y;
        if ((prev_dir.x != cur_dir.x || prev_dir.y != cur_dir.y) && (cur_slope.x > 1 || cur_slope.y > 1)) { // Problem: some slopes will have both components > 1
            if (prev_slope.x != cur_slope.x || prev_slope.y != cur_slope.y) {
                waypoints.push_back(path[wp_candidate]);
            }
            wp_candidate = i;
            prev_slope = cur_slope;
            cur_slope.x = 0;
            cur_slope.y = 0;
        }
        prev_dir = cur_dir;
    }
    if (wp_candidate != path.size() - 1) {
        waypoints.push_back(path[wp_candidate]);
    }
    waypoints.push_back(path.back());
}

void CostMap::smooth_path_v1() {
    /*
    objective: maximize the length of each segment while preventing it from representing a curved section of the path
    1 waypoint when: direction changes
    */
    if (path.empty()) return;
    waypoints.push_back(path[0]);
    if (path.size() == 1) return;
    int dx = path[1].x - path[0].x;
    int dy = path[1].y - path[0].y;
    point direction = { this, dx, dy };
    for (int i = 1; i < path.size(); i++) {
        dx = path[i].x - path[i-1].x;
        dy = path[i].y - path[i-1].y;
        if (direction.x != dx || direction.y != dy || i == path.size() - 1) {
            direction.x = dx;
            direction.y = dy;
            waypoints.push_back(path[i]);
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
    smooth_path();
    print_path();
    print_search_map();
    return waypoints;
}

// search results: 0 = untouched, 1 = added to border (evaluating cost), 2 = visited (cost evaluated), 3 = path (minimum cost)
int CostMap::output_search(point p) {
    //if (astar_data[p.x][p.y].visited == 2) return 3;
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
        astar_data[pt.x][pt.y].added = 3;
        //cout << pt.x << ',' << pt.y << " (cost = " << astar_data[pt.x][pt.y].path_cost << ")\n";
    }
    for (point pt : waypoints) {
        astar_data[pt.x][pt.y].added = 4;
        cout << pt.x << ',' << pt.y << " (cost = " << astar_data[pt.x][pt.y].path_cost << ")\n";
    }
}
