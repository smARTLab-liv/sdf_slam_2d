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
#include <fstream>
#include "../utility/UtilityFunctions.h"
#include "ros/ros.h"
#include "visualization_msgs/Marker.h"


namespace sdfslam{

    class SDFVectorMap : public AbstractMap {

    public:

        SDFVectorMap(){
            ros::NodeHandle private_nh("~");
            private_nh.param("p_grid_res_", p_grid_res_, 0.05);
            private_nh.param("p_map_size_y_", p_map_size_y_, 15);
            p_map_size_y_ /= p_grid_res_;
            private_nh.param("p_map_size_x_", p_map_size_x_, 15);
            p_map_size_x_ /= p_grid_res_;
            private_nh.param("p_min_range_", p_min_range_, 0.15);
            private_nh.param("p_max_range_", p_max_range_, 5.6);
            private_nh.param("p_avg_range_", p_avg_range_, 0);
            private_nh.param("p_update_map_aoe_", p_update_map_aoe_, 5);
            private_nh.param("p_avg_mapping_", p_avg_mapping_, false);
            private_nh.param<std::string>("p_fixed_frame_", p_fixed_frame_, "/map");


            marker_pub_ = nh_.advertise<visualization_msgs::Marker>("sdf_wall_marker", 10);

            VecMapFloat sdf((unsigned long) p_map_size_x_, std::vector<float>((unsigned long) p_map_size_y_, 0.0));
            sdf_ = sdf;

            VecMapInt sdf_level((unsigned long) p_map_size_x_, std::vector<int>((unsigned long) p_map_size_y_, 0));
            sdf_level_ = sdf_level;

            VecMapInt sdf_count((unsigned long) p_map_size_x_, std::vector<int>((unsigned long) p_map_size_y_, 0));
            sdf_count_ = sdf_count;

            epsilon_ = 0.0000001;
            std::cout << "Map constructor finished" << std::endl;
        }


        ~SDFVectorMap(){
        }

        void publish_map(){
            visualization_msgs::Marker cube_list, line_list, sphere_list;
            sphere_list.header.frame_id = cube_list.header.frame_id = line_list.header.frame_id = p_fixed_frame_;
            sphere_list.header.stamp = cube_list.header.stamp = line_list.header.stamp = ros::Time::now();
            sphere_list.ns = cube_list.ns = line_list.ns = "sdf_wall_marker";
            sphere_list.pose.orientation.w = cube_list.pose.orientation.w = line_list.pose.orientation.w = 1.0;
            sphere_list.action = cube_list.action = line_list.action = visualization_msgs::Marker::ADD;

            sphere_list.id = 1;
            cube_list.id = 1;
            line_list.id = 2;

            cube_list.type = visualization_msgs::Marker::CUBE_LIST;
            line_list.type = visualization_msgs::Marker::LINE_LIST;
            sphere_list.type = visualization_msgs::Marker::SPHERE_LIST;

            cube_list.scale.x = 2 * p_grid_res_;
            cube_list.scale.y = 2 * p_grid_res_;
            cube_list.scale.z = p_grid_res_ / 2;

            line_list.scale.x = p_grid_res_;

            sphere_list.scale.x = p_grid_res_;
            sphere_list.scale.y = p_grid_res_;

            line_list.color.r = 0.0;
            line_list.color.g = 0.0;
            line_list.color.b = 0.0;
            line_list.color.a = 1.0;

            sphere_list.color.r = 0.0;
            sphere_list.color.g = 0.0;
            sphere_list.color.b = 0.0;
            sphere_list.color.a = 1.0;

            cube_list.color.r = 0.5;
            cube_list.color.g = 0.5;
            cube_list.color.b = 0.5;
            cube_list.color.a = 1;

            //todo code copy..
            int indMin[2];
            float intensities[4];
            int indices[2];
            for (int i = 0; i < p_map_size_x_ - 1; i++)
                for (int j = 0; j < p_map_size_y_ - 1; j++) {

                    indMin[0] = i;
                    indMin[1] = j;

                    for (int k = 0; k < 4; k++) {
                        indices[0] = indMin[0] + k % 2;
                        indices[1] = indMin[1] + k / 2;
                        if (indices[0] >= 0 && indices[0] <= p_map_size_x_ && indices[1] >= 0 && indices[1] <= p_map_size_y_)
                            intensities[k] = sdf_[indices[1]][indices[0]];
                        else {
                            ROS_ERROR("map too small, inds %d %d map size %d %d", indices[0], indices[1], p_map_size_x_,
                                      p_map_size_x_);
                        }
                    }

                    bool allZero = true;
                    for (int count = 0; count < 4; count++)
                        if (intensities[count] != 0) {
                            allZero = false;
                        }
                    if (allZero) {
                        continue;
                    }
                    //check for sgn changes NESW
                    //203
                    //3x1
                    //021
                    int numChanges = 0;
                    float px[4];
                    float py[4];
                    if ((intensities[2] < 0 && intensities[3] > 0) || (intensities[3] < 0 && intensities[2] > 0)) {
                        px[numChanges] = -intensities[2] / (intensities[3] - intensities[2]);
                        py[numChanges] = 1;
                        numChanges++;
                    }
                    if ((intensities[1] < 0 && intensities[3] > 0) || (intensities[3] < 0 && intensities[1] > 0)) {
                        px[numChanges] = 1;
                        py[numChanges] = -intensities[1] / (intensities[3] - intensities[1]);
                        numChanges++;
                    }
                    if ((intensities[0] < 0 && intensities[1] > 0) || (intensities[1] < 0 && intensities[0] > 0)) {
                        px[numChanges] = -intensities[0] / (intensities[1] - intensities[0]);
                        py[numChanges] = 0;
                        numChanges++;
                    }
                    if ((intensities[0] < 0 && intensities[2] > 0) || (intensities[2] < 0 && intensities[0] > 0)) {
                        px[numChanges] = 0;
                        py[numChanges] = -intensities[0] / (intensities[2] - intensities[0]);
                        numChanges++;
                    }

                    double aPoint[2];
                    geometry_msgs::Point p;
                    if (numChanges == 2) {
                        for (int j = 0; j < numChanges; j++) {
                            aPoint[0] = indMin[0] + px[j] + 0.5;
                            aPoint[1] = indMin[1] + py[j] + 0.5;
                            util::toRl(aPoint, p_grid_res_, p_map_size_x_, p_map_size_y_);
                            p.x = aPoint[0]; //(int32_t)
                            p.y = aPoint[1];
                            //p.x = 1;
                            //p.y = 2;
                            p.z = 0;
                            line_list.points.push_back(p);
                            sphere_list.points.push_back(p);
                        }
                    }
                    else if (numChanges == 0) {
                        if (intensities[0] > 0) {
                            aPoint[0] = indMin[0] + 1;
                            aPoint[1] = indMin[1] + 1;
                            util::toRl(aPoint,p_grid_res_,p_map_size_x_,p_map_size_y_);
                            p.x = aPoint[0]; //(int32_t)
                            p.y = aPoint[1];

                            p.z = -0.1;
                            cube_list.points.push_back(p);
                        }
                    }
                    else {
                    }
                }


            marker_pub_.publish(line_list);
            marker_pub_.publish(sphere_list);
            //marker_pub_.publish(cube_list);

        }

        void update_map(const PCLPointCloud& pc, const Eigen::Vector3d& pose3d){
            //group scan endpoints
            PCLPointCloud::const_iterator it = pc.begin();
            int scan_rays = pc.width * pc.height; //todo find/use lower bound instead
            float points[scan_rays][2];
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

                    if (i >= scan_rays) {
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
        }

        void reset_map(){
            VecMapFloat sdf((unsigned long) p_map_size_x_, std::vector<float>((unsigned long) p_map_size_y_, 0.0));
            sdf_ = sdf;

            VecMapInt sdf_level((unsigned long) p_map_size_x_, std::vector<int>((unsigned long) p_map_size_y_, 0));
            sdf_level_ = sdf_level;

            VecMapInt sdf_count((unsigned long) p_map_size_x_, std::vector<int>((unsigned long) p_map_size_y_, 0));
            sdf_count_ = sdf_count;
        }

        int get_map_values(const Eigen::Vector2d& coords, float *mpdxdy, bool fine) {
            mpdxdy[0] = 0.0;
            mpdxdy[1] = 0.0;
            mpdxdy[2] = 0.0;

            float scaledCoords[2] = {coords[0], coords[1]};
            util::toMap(scaledCoords, p_grid_res_,p_map_size_x_,p_map_size_y_);
            scaledCoords[0] = scaledCoords[0] - 0.5;
            scaledCoords[1] = scaledCoords[1] - 0.5;
            int indMin[2] = {floor(scaledCoords[0]), floor(scaledCoords[1])};

            float factors[2] = {(scaledCoords[0] - indMin[0]), (scaledCoords[1] - indMin[1])};

            float intensities[4];
            int indices[2];
            for (int i = 0; i < 4; i++) {
                indices[0] = indMin[0] + i % 2;
                indices[1] = indMin[1] + i / 2;
                if (indices[0] >= 0 && indices[0] < p_map_size_x_ && indices[1] >= 0 && indices[1] < p_map_size_y_)
                    intensities[i] = sdf_[indices[1]][indices[0]];
                else {
                    ROS_ERROR("map too small, inds %d %d map size %d %d", indices[0], indices[1], p_map_size_x_, p_map_size_x_);
                    ROS_ERROR("coords %f %f", coords[0], coords[1]);
                }
            }

            //check for sgn changes NESW
            //203
            //3x1
            //021
            int numChanges = 0;
            float px[4];
            float py[4];
            if ((intensities[2] < 0 && intensities[3] > 0) || (intensities[3] < 0 && intensities[2] > 0)) {
                px[numChanges] = -intensities[2] / (intensities[3] - intensities[2]);
                py[numChanges] = 1;
                numChanges++;
            }
            if ((intensities[1] < 0 && intensities[3] > 0) || (intensities[3] < 0 && intensities[1] > 0)) {
                px[numChanges] = 1;
                py[numChanges] = -intensities[1] / (intensities[3] - intensities[1]);
                numChanges++;
            }
            if ((intensities[0] < 0 && intensities[1] > 0) || (intensities[1] < 0 && intensities[0] > 0)) {
                px[numChanges] = -intensities[0] / (intensities[1] - intensities[0]);
                py[numChanges] = 0;
                numChanges++;
            }
            if ((intensities[0] < 0 && intensities[2] > 0) || (intensities[2] < 0 && intensities[0] > 0)) {
                px[numChanges] = 0;
                py[numChanges] = -intensities[0] / (intensities[2] - intensities[0]);
                numChanges++;
            }


            bool allZero = true;
            for (int count = 0; count < 4; count++)
                if (intensities[count] != 0) {
                    allZero = false;
                }
                else
                    numChanges = 6;

            //unclean tho
            if (!fine && numChanges == 0) {
                //todo follow gradient anyways

                if (!allZero) {
                    if (intensities[0] < 0) {
                        mpdxdy[1] = -((intensities[2] - intensities[3]) * factors[1] +
                                      (1 - factors[1]) * (intensities[0] - intensities[1]));
                        mpdxdy[2] = -((intensities[0] - intensities[2]) * factors[0] +
                                      (1 - factors[0]) * (intensities[1] - intensities[3]));
                    }
                    else {
                        mpdxdy[1] = ((intensities[2] - intensities[3]) * factors[1] +
                                     (1 - factors[1]) * (intensities[0] - intensities[1]));
                        mpdxdy[2] = ((intensities[0] - intensities[2]) * factors[0] +
                                     (1 - factors[0]) * (intensities[1] - intensities[3]));
                    }
                    mpdxdy[0] = factors[1] * (factors[0] * intensities[3] + (1 - factors[0]) * intensities[2])
                                + (1 - factors[1]) * (factors[0] * intensities[1] + (1 - factors[0]) * intensities[0]);
                    mpdxdy[0] = fabs(mpdxdy[0]);
                    if (fabs(mpdxdy[1]) > fabs(mpdxdy[2])) {
                        if (mpdxdy[1] > 0)
                            mpdxdy[1] = 1;
                        else
                            mpdxdy[1] = -1;
                    }
                    else {
                        if (mpdxdy[2] > 0)
                            mpdxdy[2] = 1;
                        else
                            mpdxdy[2] = -1;
                    }

                    //cap the gradient at 1 todo experiment with that
                    if (mpdxdy[0] > p_grid_res_)
                        mpdxdy[0] = p_grid_res_;
                }
                else {
                    mpdxdy[0] = 0;
                    mpdxdy[1] = 0;
                    mpdxdy[2] = 0;
                }
            }

            //42:
            //y = mx + b
            //           yhit
            //         p1 |
            //yCoord--2-\-|----3
            //        |  \|    |
            //xhit--------\    |
            //        | x  \   |
            //        |    |\  |
            //        O----|-\-1
            //             | p0
            //           xCoord
            //
            //todo use vector form ffs

            if (numChanges == 2) {
                bool vertical;
                float m, b;
                float hit[2];
                float yCoord, xCoord;
                if (fabs(px[1] - px[0]) < epsilon_) {
                    vertical = true;
                    hit[0] = px[0];
                    hit[1] = factors[1];
                    yCoord = hit[1];
                    xCoord = hit[0];
                }
                else {
                    if (fabs(py[1] - py[0]) < epsilon_) {
                        //horizontal
                        m = 0;
                        hit[0] = factors[0];
                        hit[1] = py[0];
                        b = py[0];
                    }
                    else {
                        m = (py[1] - py[0]) / (px[1] - px[0]);
                        b = py[0] - m * px[0];
                        hit[0] = (b - factors[1] + 1 / m * factors[0]) / (-1 / m - m);
                        hit[1] = hit[0] * m + b;
                    }
                }

                if (!vertical) {
                    mpdxdy[0] = util::p2lDist(m, b, factors); //dunno
                    yCoord = factors[0] * m + b;

                    if (!m == 0)
                        xCoord = (factors[1] - b) / m;
                    else
                        //xCoord = factors[0];
                        xCoord = 999999;
                    //todo! not sure if this makes sense, not needed for vector form anyways though

                    if (factors[0] > xCoord)
                        mpdxdy[1] = -(1-xCoord);
                        //mpdxdy[1] = -1;
                    else
                        mpdxdy[1] = xCoord;
                        //mpdxdy[1] = 1;

                    if (factors[1] > yCoord)
                        mpdxdy[2] = -(1-yCoord);
                        //mpdxdy[2] = -1;
                    else
                        mpdxdy[2] = yCoord;
                        //mpdxdy[2] = 1;

                    if (yCoord < 0 || yCoord > 1)
                        mpdxdy[2] = 0;

                    if (xCoord < 0 || xCoord > 1)
                        mpdxdy[1] = 0;

                }
                else {
                    //todo replace cases with 1 solution..
                    // ROS_WARN("VERTICAL");
                    if (factors[0] < px[0])
                        mpdxdy[1] = px[0];
                        //mpdxdy[1] = 1;
                    else
                        mpdxdy[1] = px[0]-1;
                        //mpdxdy[1] = -1;

                    mpdxdy[0] = fabs(factors[0] - px[0]);
                    mpdxdy[2] = 0;
                }

                mpdxdy[0] = mpdxdy[0] / (1 / p_grid_res_);

                //mpdxdy[0] = util::round(mpdxdy[0], 32);
                //mpdxdy[1] = util::round(mpdxdy[1], 32);
                //mpdxdy[2] = util::round(mpdxdy[2], 32);

                if (mpdxdy[1] < epsilon_ && mpdxdy[1] > -epsilon_) {
                    //ROS_ERROR("asxaxaxax");
                    mpdxdy[1] = 0;
                }
                if (mpdxdy[2] < epsilon_ && mpdxdy[2] > -epsilon_) {
                    //ROS_ERROR("asxaxaxax");
                    mpdxdy[2] = 0;
                }

            }

            if (numChanges == 4) {
                mpdxdy[1] = 0;
                mpdxdy[2] = 0;
                mpdxdy[0] = 0;
            }

            return numChanges / 2;
        }

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
                            dist_temp = util::p2lsDistTwo(p_grid_res_/2, reg_border[0], reg_border[1], cell_coords) * p_grid_res_;

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
                            else { //todo might be fishy here...
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


        void save_map(std::string filename) {
            std::fstream os(filename.c_str(), std::ios::out | std::ios::app);

            for (int i = 0; i < p_map_size_y_; ++i) {
                for (int j = 0; j < p_map_size_x_; ++j) {
                    os << sdf_[i][j] << " ";
                }
                os << "\n";
            }

            os.close();
        }


        void load_map(std::string filename) {
            std::string word;
            std::ifstream myfile(filename.c_str());
            if (myfile.is_open()) {
                int xC = 0;
                int yC = 0;
                while (myfile >> word) {
                    sdf_[xC][yC] = (float) atof(word.c_str());
                    if (xC < p_map_size_y_ - 1)
                        xC++;
                    else {
                        xC = 0;
                        yC++;
                    }
                }
                myfile.close();
            }
            else
                ROS_ERROR("Unable to open file");
        }

/*
          void publish_map_cheap(bool updated) {
              if (updated) {
                  iso_map_.clear();
                  //sdf_thresholded_.clear();
                  cloud_map_.clear();

                  int counter = 0;
                  for (int i = 1; i < sdf_thresholded_.size() - 1; i++)
                      for (int j = 1; j < sdf_thresholded_[0].size() - 1; j++)
                          if (sdf_[i][j] < -p_isovalue_)
                              sdf_thresholded_[i][j] = 1;
                          else
                              sdf_thresholded_[i][j] = 0;


                  for (int i = 1; i < sdf_thresholded_.size() - 1; i++) {
                      for (int j = 1; j < sdf_thresholded_[0].size() - 1; j++) {
                          if (sdf_thresholded_[i][j] == 1) {
                              Eigen::Vector2f coord(j + 0.5, i + 0.5);
                              coord = util::toRl(coord, p_grid_res_, p_map_size_x_, p_map_size_y_);
                              iso_map_.push_back(util::genPoint(coord.x(), coord.y()));
                              counter++;
                          }
                          //}//del
                          //}//del if unmcomment
                          //}//del if

                          int state = 0;
                          if (sdf_thresholded_[i][j] == 1)
                              state += 8;
                          if (sdf_thresholded_[i][j + 1] == 1)
                              state += 4;
                          if (sdf_thresholded_[i - 1][j + 1] == 1)
                              state += 2;
                          if (sdf_thresholded_[i - 1][j] == 1)
                              state += 1;

                          // double yPos = (i+0.5-p_map_size_y_/2)*p_grid_res_;
                          // double xPos = (j+0.5-p_map_size_x_/2)*p_grid_res_;
                          Eigen::Vector2f coord(j, i);
                          coord = util::toRl(coord, p_grid_res_, p_map_size_x_, p_map_size_y_);
                          double yPos = coord.y() + p_grid_res_ / 2;//(i-p_map_size_y_/2)*p_grid_res_;
                          double xPos = coord.x() + p_grid_res_ / 2;//(j-p_map_size_x_/2)*p_grid_res_;

                          // cloud_map_.width = counter * 6;
                          // cloud_map_.height   = 1;
                          // cloud_map_.is_dense = false;
                          // //	cloud_map_.points.resize(cloud_map_.width * cloud_map_.height);

                          switch (state) {
                              case 0:
                                  break;

                              case 1:
                                  cloud_map_.push_back(util::genPoint(xPos, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos - p_grid_res_));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 4, yPos - 3 * p_grid_res_ / 4));
                                  break;

                              case 2:
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos - p_grid_res_));
                                  cloud_map_.push_back(util::genPoint(xPos + 3 * p_grid_res_ / 4, yPos - 3 * p_grid_res_ / 4));
                                  break;

                              case 3:
                                  cloud_map_.push_back(util::genPoint(xPos, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_, yPos - p_grid_res_ / 2));
                                  break;

                              case 4:
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + 3 * p_grid_res_ / 4, yPos - p_grid_res_ / 4));
                                  break;

                              case 5:
                                  cloud_map_.push_back(util::genPoint(xPos, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 4, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos - p_grid_res_));
                                  cloud_map_.push_back(util::genPoint(xPos + 3 * p_grid_res_ / 4, yPos - 3 * p_grid_res_ / 4));
                                  break;

                              case 6:
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos - p_grid_res_));
                                  break;

                              case 7:
                                  cloud_map_.push_back(util::genPoint(xPos, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 4, yPos - p_grid_res_ / 2));
                                  break;

                              case 8:
                                  cloud_map_.push_back(util::genPoint(xPos, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 4, yPos - p_grid_res_ / 2));
                                  break;

                              case 9:
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos - p_grid_res_));
                                  break;

                              case 10:
                                  cloud_map_.push_back(util::genPoint(xPos, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos - p_grid_res_));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 4, yPos - 3 * p_grid_res_ / 4));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + 3 * p_grid_res_ / 4, yPos - p_grid_res_ / 4));
                                  break;

                              case 11:
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + 3 * p_grid_res_ / 4, yPos - p_grid_res_ / 4));
                                  break;

                              case 12:
                                  cloud_map_.push_back(util::genPoint(xPos, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_, yPos - p_grid_res_ / 2));
                                  break;

                              case 13:
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos - p_grid_res_));
                                  cloud_map_.push_back(util::genPoint(xPos + 3 * p_grid_res_ / 4, yPos - 3 * p_grid_res_ / 4));
                                  break;

                              case 14:
                                  cloud_map_.push_back(util::genPoint(xPos, yPos - p_grid_res_ / 2));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 2, yPos - p_grid_res_));
                                  cloud_map_.push_back(util::genPoint(xPos + p_grid_res_ / 4, yPos - 3 * p_grid_res_ / 4));
                                  break;

                              case 15:
                                  break;

                              default:
                                  ROS_WARN("this should not have happened, switch @ publish clud_map_ broken");
                                  break;
                          }
                      }

                  }
              }
              sensor_msgs::PointCloud2 cloudMsg;
              pcl::toROSMsg(cloud_map_, cloudMsg);
              cloudMsg.header.frame_id = p_fixed_frame_;
              cloudMsg.header.stamp = ros::Time::now();
              map_pub_.publish(cloudMsg);

              //rm me
              sensor_msgs::PointCloud2 isoMsg;
              pcl::toROSMsg(iso_map_, isoMsg);
              isoMsg.header.frame_id = p_fixed_frame_;
              isoMsg.header.stamp = ros::Time::now();
              iso_pub_.publish(isoMsg); //rm me

              pcl::PointCloud<pcl::PointXYZ>::Ptr monster_cloud(new pcl::PointCloud<pcl::PointXYZ>);
              int insertAt = 0;
              monster_cloud->width = p_map_size_x_ * p_map_size_y_;
              monster_cloud->height = 1;
              monster_cloud->is_dense = false;
              monster_cloud->points.resize(monster_cloud->width * monster_cloud->height);
              for (int i = 0; i < p_map_size_y_; i++) {
                  for (int j = 0; j < p_map_size_x_; j++) {
                      if (sdf_[i][j] != 0 && sdf_[i][j] < p_truncation_ && sdf_[i][j] > -p_truncation_) {
                          monster_cloud->points[insertAt].z = sdf_[i][j];
                          Eigen::Vector2f coord(j, i);
                          coord = util::toRl(coord, p_grid_res_, p_map_size_x_, p_map_size_y_);
                          monster_cloud->points[insertAt].x = p_grid_res_ / 2 + j * p_grid_res_ - (p_map_size_x_ / 2) * p_grid_res_;
                          //monster_cloud->points[insertAt].x = coord.x()+p_grid_res_/2;
                          monster_cloud->points[insertAt].y = p_grid_res_ / 2 + i * p_grid_res_ - (p_map_size_y_ / 2) * p_grid_res_;
                          //monster_cloud->points[insertAt].y = coord.y()+p_grid_res_/2;
                          insertAt++;
                      }
                  }
              }
              sensor_msgs::PointCloud2 cloudMsg2;
              pcl::toROSMsg(*monster_cloud, cloudMsg2);
              cloudMsg2.header.frame_id = p_fixed_frame_;
              cloudMsg2.header.stamp = ros::Time::now();
              cloudA_pub_.publish(cloudMsg2);

          }
*/

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

        ros::NodeHandle nh_;

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
        ros::Publisher marker_pub_;
        std::string p_fixed_frame_;

/*
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_map", 10);
        iso_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("iso_map", 10);
        cloudA_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloudA", 10);
        ros::Publisher map_pub_;
        ros::Publisher iso_pub_;
        ros::Publisher cloudA_pub_;
*/
    };

}

#endif //SDF_SLAM_2D_VECTORMAP_H
