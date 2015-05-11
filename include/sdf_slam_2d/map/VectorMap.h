//=================================================================================================
//Copyright (C) 2015, Joscha Fossel
//
//This program is free software; you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation; either version 2 of the License, or
//any later version.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License along
//with this program; if not, write to the Free Software Foundation, Inc.,
//51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//=================================================================================================

#ifndef SDF_SLAM_2D_VECTORMAP_H
#define SDF_SLAM_2D_VECTORMAP_H

#include "AbstractMap.h"

#include <iostream>
#include "../utility/UtilityFunctions.h"
#include "ros/ros.h"

namespace sdfslam{

    class SDFVectorMap : public AbstractMap {

    public:

        SDFVectorMap(){
            ros::NodeHandle private_nh("~");
            private_nh.param("p_grid_res_", p_grid_res_, 0.05);
            private_nh.param("p_map_size_y_", p_map_size_y_, 15);
            private_nh.param("p_map_size_x_", p_map_size_x_, 15);
            private_nh.param("p_min_range_", p_min_range_, 0.15);
            private_nh.param("p_max_range_", p_max_range_, 5.6);
            private_nh.param("p_avg_range_", p_avg_range_, 0);
            private_nh.param("p_update_map_aoe_", p_update_map_aoe_, 5);
            private_nh.param("p_avg_mapping_", p_avg_mapping_, false);


            VecMapFloat sdf((unsigned long) p_map_size_x_, std::vector<float>((unsigned long) p_map_size_y_, 0.0));
            sdf_ = sdf;

            VecMapInt sdf_level((unsigned long) p_map_size_x_, std::vector<int>((unsigned long) p_map_size_y_, 0));
            sdf_level_ = sdf_level;

            VecMapInt sdf_count((unsigned long) p_map_size_x_, std::vector<int>((unsigned long) p_map_size_y_, 0));
            sdf_count_ = sdf_count;

            epsilon_ = 0.0000001;
            std::cout << "Map constructor finished" << std::endl;
        };


        ~SDFVectorMap(){
        };


        bool update_map(const PCLPointCloud& pc, const Eigen::Vector3f& pose3d){
            std::cout << "Map update called" << std::endl;
            //group scan endpoints

            PCLPointCloud::const_iterator it = pc.begin();
            int p_scan_rays_ = pc.width * pc.height; //todo find/use lower bound instead
            float points[p_scan_rays_][2];
            float a_point_in_rl[2];
            float a_point_in_map[2];
            float point_window[2];
            float pos[2] = {pose3d[0], pose3d[1]};
            bool cont = true;
            int i = 0;
            float next_point_window[2];

            while (it < pc.end()) {

                //todo max range
                a_point_in_rl[0] = it->x;
                a_point_in_rl[1] = it->y;

                //todo put this somewhere where dist is calculated anyway
                if (util::p2pDist(a_point_in_rl, pos) < p_min_range_) {
                    it++;
                    continue;
                }
                else if (util::p2pDist(a_point_in_rl, pos) > p_max_range_) {
                    it++;
                    continue;
                }

                a_point_in_map[0] = a_point_in_rl[0];
                a_point_in_map[1] = a_point_in_rl[1];
                util::toMap(a_point_in_map,p_grid_res_,p_map_size_x_,p_map_size_y_);

                if (fmod(a_point_in_map[0], 1) < 0.5)
                    point_window[0] = floor(a_point_in_map[0]) - 0.5;
                else
                    point_window[0] = floor(a_point_in_map[0]) + 0.5;
                if (fmod(a_point_in_map[1], 1) < 0.5)
                    point_window[1] = floor(a_point_in_map[1]) - 0.5;
                else
                    point_window[1] = floor(a_point_in_map[1]) + 0.5;

                cont = true;
                i = 0;
                while (cont) {

                    if (i >= p_scan_rays_) {
                        ROS_ERROR("TOO MANY SCAN ENDPOINTS, THIS SHOULD NOT HAVE HAPPENED, VectorMap.h");
                    }

                    points[i][0] = it->x;
                    points[i][1] = it->y;
                    i++;
                    it++;
                    if (it < pc.end()) {
                        a_point_in_rl[0] = it->x;
                        a_point_in_rl[1] = it->y;
                        a_point_in_map[0] = a_point_in_rl[0];
                        a_point_in_map[1] = a_point_in_rl[1];
                        util::toMap(a_point_in_map,p_grid_res_,p_map_size_x_,p_map_size_y_);

                        if (fmod(a_point_in_map[0], 1) < 0.5)
                            next_point_window[0] = floor(a_point_in_map[0]) - 0.5;
                        else
                            next_point_window[0] = floor(a_point_in_map[0]) + 0.5;

                        if (fmod(a_point_in_map[1], 1) < 0.5)
                            next_point_window[1] = floor(a_point_in_map[1]) - 0.5;
                        else
                            next_point_window[1] = floor(a_point_in_map[1]) + 0.5;

                        //todo this is not always correct, e.g. corner case:
                        //___\_
                        //|   x|2
                        //|    \
                        //|    |\
                        //|_x__|x\
                        //  0  |1
                        if (!(next_point_window[0] == point_window[0] && next_point_window[1] == point_window[1])) {
                            cont = false;
                        }
                    }
                    else
                        cont = false;
                }

                //more than one point in window
                if (i > 1)
                    update_cells(pos, points, i, point_window, sdf_);
                else {
                    //todo rm hack, look at neighbour scans (see note above..), also, introduce distance threshold for neigbours, if exceeded num=1...
                    if ((it - 1) > pc.begin() && (it) < pc.end()) {
                        if (util::p2pDist((it - 2)->x, (it - 2)->y, points[0]) < util::p2pDist(it->x, it->y, points[0])) {
                            points[i][0] = (it - 2)->x;
                            points[i][1] = (it - 2)->y;
                        }
                        else {
                            points[i][0] = it->x;
                            points[i][1] = it->y;
                        }
                    }
                    else if (it - 1 > pc.begin()) {
                        points[i][0] = (it - 2)->x;
                        points[i][1] = (it - 2)->y;
                    }
                    else if (it < pc.end()) {
                        points[i][0] = it->x;
                        points[i][1] = it->y;
                    }
                    update_cells(pos, points, 2, point_window, sdf_);
                }
            }
        };

        int get_map_gradients(){

        };

    protected:

        void deming_regression(float tars[][2], int num, double &beta0, double &beta1) {
            //todo hackhack, use neighbors
            bool revert = false;
            if (num == 0) {
                num = 2;
                revert = true;
            }

            //orthogonal deming regression
            double xbar = 0;
            double ybar = 0;
            double sxx = 0;
            double sxy = 0;
            double syy = 0;
            for (int i = 0; i < num; i++) {
                xbar += util::toMap(tars[i][0], p_grid_res_, p_map_size_x_);
                ybar += util::toMap(tars[i][1], p_grid_res_, p_map_size_y_);
            }
            xbar /= num;
            ybar /= num;
            for (int i = 0; i < num; i++) {
                sxx += (util::toMap(tars[i][0], p_grid_res_, p_map_size_x_) - xbar) * (util::toMap(tars[i][0], p_grid_res_, p_map_size_x_) - xbar);
                sxy += (util::toMap(tars[i][0], p_grid_res_, p_map_size_x_) - xbar) * (util::toMap(tars[i][1], p_grid_res_, p_map_size_y_) - ybar);
                syy += (util::toMap(tars[i][1], p_grid_res_, p_map_size_y_) - ybar) * (util::toMap(tars[i][1], p_grid_res_, p_map_size_y_) - ybar);
            }
            sxx /= num - 1;
            sxy /= num - 1;
            syy /= num - 1;

            beta1 = (syy - sxx + sqrt((syy - sxx) * (syy - sxx) + 4 * sxy * sxy)) / (2 * sxy);
            beta0 = ybar - beta1 * xbar;
            //y=beta0+beta1*x
            //end deming

            if (revert)
                num = 1;
        }

        void update_cells(float *src, float tars[][2], int num, float *window, VecMapFloat &aMap) {
            {
                float srcInMap[2] = {src[0], src[1]};
                util::toMap(srcInMap, p_grid_res_, p_map_size_x_, p_map_size_y_);
                float tarInMap[2] = {tars[0][0], tars[0][1]};
                util::toMap(tarInMap, p_grid_res_, p_map_size_x_, p_map_size_y_);

                double beta0, beta1;

                bool horizontal = true;
                for (int i = 0; i < num; i++) {
                    //ROS_INFO("i %d/%d comparing %f %f", i, num,tars[0][1], tars[i][1]);
                    if (fabs(tars[0][1] - tars[i][1]) > epsilon_) {
                        horizontal = false;
                        break;
                    }
                }

                //ROS_ERROR("horizontal %d", horizontal);

                if (!horizontal)
                    deming_regression(tars, num, beta0, beta1);
                else {
                    beta1 = 0;
                    beta0 = tarInMap[1];
                }

                if (beta0 != beta0 || beta1 != beta1) {
                    ROS_ERROR("deming says %f %f (adjust epsilon)", beta0, beta1);
                    return;
                }

                int x_index = floor(window[0]) - p_update_map_aoe_;
                int y_index = floor(window[1]) - p_update_map_aoe_;

                float reg_border[2][2];
                int reg_border_counter = 0;

                double x_temp;
                double y_temp;
                for (int reg_counter = 0; reg_counter < 2; reg_counter++) {
                    if (beta1 != 0) {
                        x_temp = (window[1] + reg_counter - beta0) / beta1;
                        if (x_temp >= window[0] && x_temp <= window[0] + 1) {
                            reg_border[reg_border_counter][0] = x_temp;
                            reg_border[reg_border_counter][1] = window[1] + reg_counter;
                            reg_border_counter++;
                        }
                    }

                    y_temp = beta0 + beta1 * (window[0] + reg_counter);
                    if (y_temp >= window[1] && y_temp <= window[1] + 1) {
                        reg_border[reg_border_counter][0] = window[0] + reg_counter;;
                        reg_border[reg_border_counter][1] = y_temp;
                        reg_border_counter++;
                    }
                }


                bool vertical = false;
                if (beta1 > 999999 || beta1 < -999999) { //vertical
                    reg_border[0][0] = tarInMap[0];
                    reg_border[0][1] = window[1];
                    reg_border[1][0] = tarInMap[0];
                    reg_border[1][1] = window[1] + 1;
                    vertical = true;
                }

                int level = p_update_map_aoe_;
                int new_x_index;
                int new_y_index;
                float cell_coords[2];
                bool invert;
                float dist_temp;
                float mapVal_temp;
                for (int counter_x = 0; counter_x < p_update_map_aoe_ * 2 + 2; counter_x++) {
                    for (int counter_y = 0; counter_y < p_update_map_aoe_ * 2 + 2; counter_y++) {
                        level = std::max(abs(trunc(counter_x - p_update_map_aoe_ - 0.5)),
                                         abs(trunc(counter_y - p_update_map_aoe_ - 0.5)));

                        new_x_index = x_index + counter_x;
                        new_y_index = y_index + counter_y;
                        cell_coords[0] = new_x_index + 0.5;
                        cell_coords[1] = new_y_index + 0.5;

                        if (new_x_index >= p_map_size_x_ || new_y_index >= p_map_size_y_)
                            ROS_ERROR("out of map bounds");

                        invert = invertCheck(srcInMap, cell_coords, reg_border);

                        if (vertical) {
                            if (cell_coords[1] >= window[1] - epsilon_ && cell_coords[1] <= window[1] + 1 + epsilon_)
                                dist_temp = ((float) (fabs(cell_coords[0] - reg_border[0][0]) * p_grid_res_));
                            else
                                dist_temp = 0.0;
                        }
                        else if (level == 0)
                            dist_temp = util::p2lsDist(reg_border[0], reg_border[1], cell_coords) * p_grid_res_;
                        else
                            dist_temp = util::p2lsDistTwo(0, reg_border[0], reg_border[1], cell_coords) * p_grid_res_;

                        if (dist_temp != 0.0) {
                            if (invert)
                                dist_temp = -dist_temp;

                            if (sdf_count_[new_y_index][new_x_index] == 0) {
                                aMap[new_y_index][new_x_index] = dist_temp;
                                sdf_level_[new_y_index][new_x_index] = level;
                            }
                            else if (aMap[new_y_index][new_x_index] * dist_temp <
                                     0) { //todo not fast either, also use level for something useful...
                                //if (dist_temp>0 || level==0){
                                //if (sdf_level_[new_y_index][new_x_index] > level || (sdf_level_[new_y_index][new_x_index] == level && dist_temp > 0))  {
                                //if (sdf_level_[new_y_index][new_x_index] > level || (level > p_update_map_aoe_ && sdf_level_[new_y_index][new_x_index] == level && dist_temp > 0))  {
                                if (sdf_level_[new_y_index][new_x_index] > level) {
                                    aMap[new_y_index][new_x_index] = dist_temp;
                                    sdf_level_[new_y_index][new_x_index] = level;
                                }
                            }
                            else {
                                if (sdf_level_[new_y_index][new_x_index] == level ||
                                    sdf_level_[new_y_index][new_x_index] == level - p_avg_range_) {
                                    if (!p_avg_mapping_) {
                                        if (!invert)
                                            aMap[new_y_index][new_x_index] = fmin(aMap[new_y_index][new_x_index], dist_temp);
                                        else
                                            aMap[new_y_index][new_x_index] = fmax(aMap[new_y_index][new_x_index], dist_temp);
                                    }
                                    else {
                                        mapVal_temp = (aMap[new_y_index][new_x_index] * sdf_count_[new_y_index][new_x_index]
                                                       + dist_temp) / (sdf_count_[new_y_index][new_x_index] + 1);
                                    }
                                }
                                else if (aMap[new_y_index][new_x_index] > level) {
                                    aMap[new_y_index][new_x_index] = dist_temp;
                                }
                                sdf_level_[new_y_index][new_x_index] = level;
                            }
                            sdf_count_[new_y_index][new_x_index] = sdf_count_[new_y_index][new_x_index] + 1;
                        }
                    }
                }
            }
        }

        bool invertCheck(float *src, float *tar, float border[2][2]) {
            float x1 = src[0];
            float y1 = src[1];
            float x2 = tar[0];
            float y2 = tar[1];
            float x3 = border[0][0];
            float y3 = border[0][1];
            float x4 = border[1][0];
            float y4 = border[1][1];

            float sx = ((x4 - x3) * (x2 * y1 - x1 * y2) - (x2 - x1) * (x4 * y3 - x3 * y4)) /
                       ((y4 - y3) * (x2 - x1) - (y2 - y1) * (x4 - x3));
            float sy = ((y1 - y2) * (x4 * y3 - x3 * y4) - (y3 - y4) * (x2 * y1 - x1 * y2)) /
                       ((y4 - y3) * (x2 - x1) - (y2 - y1) * (x4 - x3));

            float src_tar_dist = util::p2pDist(src, tar);
            float src_border_dist = util::p2pDist(sx, sy, src);

            if ((tar[0] - src[0] > 0 && sx - src[0] < 0) ||
                (tar[0] - src[0] < 0 && sx - src[0] > 0) ||
                (src_border_dist > src_tar_dist))
                return false;
            else
                return true;
        }

        VecMapFloat sdf_;
        VecMapInt sdf_level_;
        VecMapInt sdf_count_;
        double p_grid_res_;
        int p_map_size_x_;
        int p_map_size_y_;
        double p_min_range_;
        double p_max_range_;
        double epsilon_;
        int p_avg_range_;
        bool p_avg_mapping_;
        int p_update_map_aoe_;


    };

}

#endif //SDF_SLAM_2D_VECTORMAP_H
