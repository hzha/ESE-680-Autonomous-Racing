#include "scan_matching_skeleton/correspond.h"
#include "cmath"
#include "ros/ros.h"

using namespace std;

const int UP_SMALL = 0;
const int UP_BIG = 1;
const int DOWN_SMALL = 2;
const int DOWN_BIG = 3;

void getNaiveCorrespondence(vector<Point> &old_points, vector<Point> &trans_points, vector<Point> &points,
                            vector<vector<int> > &jump_table, vector<Correspondence> &c, float prob) {

    c.clear();
    int last_best = -1;
    const int n = trans_points.size();
    const int m = old_points.size();

    int min_index = 0;
    int second_min_index = 0;

    //Do for each point
    for (int i = 0; i < n; ++i) {
        float min_dist = 100000.00;

        for (int j = 0; j < m; ++j) {
            float dist = old_points[j].distToPoint2(&trans_points[i]);
            if (dist < min_dist) {
                min_dist = dist;
                min_index = j;
                second_min_index = min_index <= 0 ? j + 1 : j - 1;

            }
        }
        c.push_back(
                Correspondence(&trans_points[i], &points[i], &old_points[min_index], &old_points[second_min_index]));
    }


}

void getCorrespondence(vector<Point> &old_points, vector<Point> &trans_points, vector<Point> &points,
                       vector<vector<int> > &jump_table, vector<Correspondence> &c, float prob) {

    // Written with inspiration from: https://github.com/AndreaCensi/gpc/blob/master/c/gpc.c
    // use helper functions and structs in transform.h and correspond.h
    // input : old_points : vector of struct points containing the old points (points of the previous frame)
    // input : trans_points : vector of struct points containing the new points transformed to the previous frame using the current estimated transform
    // input : points : vector of struct points containing the new points
    // input : jump_table : jump table computed using the helper functions from the transformed and old points
    // input : c: vector of struct correspondences . This is a refernece which needs to be updated in place and return the new correspondences to calculate the transforms.
    // output : c; update the correspondence vector in place which is provided as a reference. you need to find the index of the best and the second best point.
    //Initializecorrespondences



    c.clear();
    int last_best = -1;
    const int n = trans_points.size();
    const int m = old_points.size();
    const int nrays = trans_points.size();//
    //Do for each point
    for (int i = 0; i < n; ++i) {

        int best = 0;
        int second_best = 0;


        int start_index = (trans_points[i].theta - old_points[0].theta) * (nrays / (2 * M_PI));//.theta or .r??????????
        if (start_index <= 0)
            start_index = 0;
        else if (start_index >= m)
            start_index = m - 1;

        int we_start_at = (last_best != -1) ? (last_best + 1) : start_index;
        int up = we_start_at + 1;
        int down = we_start_at;

//      double last_dist_up = numeric_limits<double>::infinity();
//      double last_dist_down = numeric_limits<double>::infinity();
//      int best_distance = numeric_limits<double>::infinity();
        double last_dist_up = 100000;
        double last_dist_down = 1000001;
        double best_distance = DBL_MAX;

        bool up_stopped = false, down_stopped = false;

        while (!(up_stopped && down_stopped)) {
            bool now_up = !up_stopped && (last_dist_up < last_dist_down);

            if (now_up) {
                if (up >= m) {
                    up_stopped = true;
                    continue;
                }
                last_dist_up = trans_points[i].distToPoint2(&old_points[up]);
                if (last_dist_up < best_distance) {
                    best = up;
                    best_distance = last_dist_up;
                }
                if (up > start_index) {
                    double delta_angle_up = old_points[up].theta - trans_points[i].theta;
                    double min_dist_up = sin(delta_angle_up) * trans_points[i].r;

                    if (pow(min_dist_up, 2) > best_distance) {
                        up_stopped = true;
                        continue;
                    }

                    up = (old_points[up].r < trans_points[i].r) ? jump_table[up][UP_BIG] : jump_table[up][UP_SMALL];
                } else
                    up++;


            }

            if (!now_up) {
                if (down <= -1) {
                    down_stopped = true;
                    continue;
                }
                last_dist_down = trans_points[i].distToPoint2(&old_points[down]);

                if (last_dist_down < best_distance) {
                    best = down;
                    best_distance = last_dist_down;
                }
                if (down < start_index) {
                    double delta_angle_down = -old_points[down].theta + trans_points[i].theta;
                    double min_dist_down = sin(delta_angle_down) * trans_points[i].r;
                    if (pow(min_dist_down, 2) > best_distance) {
                        down_stopped = true;
                        continue;
                    }
                    down = (old_points[down].r < trans_points[i].r) ? jump_table[down][DOWN_BIG]
                                                                    : jump_table[down][DOWN_SMALL];
                } else
                    down--;
            }

        }
        last_best = best;
        second_best = best > 0 ? best - 1 : best + 1;


        c.push_back(Correspondence(&trans_points[i], &points[i], &old_points[best], &old_points[second_best]));


    }
}


void computeJump(vector<vector<int> > &table, vector<Point> &points) {
    table.clear();
    int n = points.size();
    for (int i = 0; i < n; ++i) {
        vector<int> v = {n, n, -1, -1};
        for (int j = i + 1; j < n; ++j) {
            if (points[j].r < points[i].r) {
                v[UP_SMALL] = j;
                break;
            }
        }
        for (int j = i + 1; j < n; ++j) {
            if (points[j].r > points[i].r) {
                v[UP_BIG] = j;
                break;
            }
        }
        for (int j = i - 1; j >= 0; --j) {
            if (points[j].r < points[i].r) {
                v[DOWN_SMALL] = j;
                break;
            }
        }
        for (int j = i - 1; j >= 0; --j) {
            if (points[j].r > points[i].r) {
                v[DOWN_BIG] = j;
                break;
            }
        }
        table.push_back(v);
    }
}
