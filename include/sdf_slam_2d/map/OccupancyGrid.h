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

#ifndef SDF_SLAM_2D_VISUALIZATIONMAP_H
#define SDF_SLAM_2D_VISUALIZATIONMAP_H

#include "../utility/Types.h"
#include "nav_msgs/OccupancyGrid.h"

namespace sdfslam{

    class OccupancyGrid : public AbstractMap {

    public:

        OccupancyGrid(){
            ros::NodeHandle private_nh("~");
            private_nh.param("p_grid_res_", p_grid_res_, 0.05);
            private_nh.param("p_map_size_y_", p_map_size_y_, 15);
            p_map_size_y_ /= p_grid_res_;
            private_nh.param("p_map_size_x_", p_map_size_x_, 15);
            p_map_size_x_ /= p_grid_res_;
            private_nh.param<std::string>("p_fixed_frame_", p_fixed_frame_, "/map");

            occ_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occ_map", 10);

            VecMapFloat occ_map((unsigned long) p_map_size_x_, std::vector<float>((unsigned long) p_map_size_y_, 0.0));
            occ_map_ = occ_map;
        }

        void save_map(std::string filename){}

        void load_map(std::string filename){};

        void reset_map(){
            VecMapFloat occ_map((unsigned long) p_map_size_x_, std::vector<float>((unsigned long) p_map_size_y_, 0.0));
            occ_map_ = occ_map;
        };

        void publish_map(){
            std::vector<int8_t> omap2;
            for (int i = 0; i < p_map_size_x_; i++) {
                for (int j = 0; j < p_map_size_y_; j++) {
                    if (occ_map_[i][j] == 0)
                        omap2.push_back(-1);
                    else if (occ_map_[i][j] > 0)
                        omap2.push_back(0);
                    else
                        omap2.push_back(-1);
                }
            }
            nav_msgs::OccupancyGrid mapGrid;
            nav_msgs::MapMetaData metaData;

            metaData.resolution = (float) p_grid_res_;
            metaData.width = p_map_size_x_;
            metaData.height = p_map_size_y_;
            metaData.origin.position.x = -p_map_size_x_ / 2 * p_grid_res_;
            metaData.origin.position.y = -p_map_size_y_ / 2 * p_grid_res_;
            mapGrid.data = omap2;
            mapGrid.info = metaData;
            mapGrid.header.frame_id = p_fixed_frame_;
            occ_map_pub_.publish(mapGrid);
        };

        void update_map(const PCLPointCloud& pc, const Eigen::Vector3f& pose3d){
            PCLPointCloud::const_iterator it = pc.begin();
            while (it < pc.end()) {
                Eigen::Vector2d a_point(it->x, it->y);
                Eigen::Vector2d cur_pos(pose3d.x(), pose3d.y());
                simple_raytrace(cur_pos, a_point, occ_map_);
                it++;
            }
        };


    protected:

        void simple_raytrace(Eigen::Vector2d src, Eigen::Vector2d tar, VecMapFloat &aMap) {

            double x0 = src.x() / p_grid_res_ + p_map_size_x_ / 2;
            double y0 = src.y() / p_grid_res_ + p_map_size_y_ / 2;
            double x1 = tar.x() / p_grid_res_ + p_map_size_x_ / 2;
            double y1 = tar.y() / p_grid_res_ + p_map_size_y_ / 2;

            double dx = fabs(x1 - x0);
            double dy = fabs(y1 - y0);

            int x = int(floor(x0));
            int y = int(floor(y0));

            int n = 1;
            int x_inc, y_inc;
            double error;
            double distance;

            if (dx == 0) {
                x_inc = 0;
                error = std::numeric_limits<double>::infinity();
            }
            else if (x1 > x0) {
                x_inc = 1;
                n += int(floor(x1)) - x;
                error = (floor(x0) + 1 - x0) * dy;
            }
            else {
                x_inc = -1;
                n += x - int(floor(x1));
                error = (x0 - floor(x0)) * dy;
            }

            if (dy == 0) {
                y_inc = 0;
                error -= std::numeric_limits<double>::infinity();
            }
            else if (y1 > y0) {
                y_inc = 1;
                n += int(floor(y1)) - y;
                error -= (floor(y0) + 1 - y0) * dx;
            }
            else {
                y_inc = -1;
                n += y - int(floor(y1));
                error -= (y0 - floor(y0)) * dx;
            }

            for (; n > 1; --n) {
                aMap[y][x] = 1;

                if (error > 0) {
                    y += y_inc;
                    error -= dx;
                }
                else {
                    x += x_inc;
                    error += dy;
                }
            }
        }


        VecMapFloat occ_map_;
        ros::Publisher occ_map_pub_;
        double p_grid_res_;
        int p_map_size_x_;
        int p_map_size_y_;
        std::string p_fixed_frame_;
        ros::NodeHandle nh_;
    };

}

#endif //SDF_SLAM_2D_VISUALIZATIONMAP_H
