#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <algorithm>
#include <string>
#include <sstream>

using std::cout;
using std::endl;
using std::vector;
using std::string;

void travel(int cur, int next) {
    unsigned int x = 1;
    while (x != 0) {
        x++;
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) return 0;
    std::ifstream ifs(argv[1]);
    double wpc; //waypoint component
    int dim = 0; //dimension
    int i = 0; //waypoint index
    vector<vector<double>> wps(1);
    wps[0].resize(3);
    while (ifs >> wpc) {
        if (dim > 2) {
            dim = 0;
            i++;
            wps.resize(i+1);
            wps[i].resize(3);
        }
        wps[i][dim++] = wpc;
    }
    vector<double> cur = wps[0];
    cout << "starting at " << cur[0] << ", " << cur[1] << ", " << cur[2] << '\n';
    wps.erase(wps.begin());
    while (!wps.empty()) {
        int min_i = 0;
        double min = std::numeric_limits<double>::max();
        for (i = 0; i < (int)wps.size(); i++) {
            float dist = sqrt(pow(wps[i][0] - cur[0], 2) + pow(wps[i][1] - cur[1], 2) + pow(wps[i][2] - cur[2], 2));
            if (dist < min) {
                min = dist;
                min_i = i;
            }
        }
        //travel(cur, min_i);
        cur = wps[min_i];
        wps.erase(wps.begin() + min_i);
        cout << "reached " << cur[0] << ", " << cur[1] << ", " << cur[2] << '\n';
    }
}