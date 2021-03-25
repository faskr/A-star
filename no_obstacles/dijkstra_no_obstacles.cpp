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

struct dk {
    double d = 0; //distance from last point on path
    bool k = 0; //reached
};

void travel(int cur, int next) {
    uint32_t x = 1;
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
    cout << "starting at " << wps[0][0] << ", " << wps[0][1] << ", " << wps[0][2] << '\n';
    int wp_cnt = (int)wps.size();
    vector<dk> aspects(wp_cnt);
    for (int i = 0; i < wp_cnt; i++) {
        aspects[i].d = std::numeric_limits<double>::infinity();
        aspects[i].k = false;
    }
    aspects[0].d = 0;
    int min_i = 0;
    int cur_i = 0;
    double min_d;
    vector<double> diff(3);
    for (int i = 0; i < wp_cnt; i++) {
        min_d = std::numeric_limits<double>::infinity();
        for (int j = 0; j < wp_cnt; j++) {
            if (aspects[j].k) continue;
            if (aspects[j].d < min_d) {
                min_d = aspects[j].d;
                min_i = j;
            }
        }
        aspects[min_i].k = true;
        if (i > 0) {
            travel(cur_i, min_i);
            cur_i = min_i;
            cout << "reached " << wps[min_i][0] << ", " << wps[min_i][1] << ", " << wps[min_i][2] << '\n';
        }
        for (int j = 0; j < wp_cnt; j++) {
            if (!aspects[j].k) {
                diff[0] = (double)wps[min_i][0] - (double)wps[j][0];
                diff[1] = (double)wps[min_i][1] - (double)wps[j][1];
                diff[2] = (double)wps[min_i][2] - (double)wps[j][2];
                aspects[j].d = sqrt(pow(diff[0], 2) + pow(diff[1], 2) + pow(diff[2], 2));
            }
        }
    }
}