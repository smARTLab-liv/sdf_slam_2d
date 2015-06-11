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

//todo: Save complete map as a graph, for now, just maintain one snapshot..

#ifndef SDF_SLAM_2D_GRAPHMAP_H
#define SDF_SLAM_2D_GRAPHMAP_H

#include "VectorMap.h"

//#include <tf/transform_listener.h>
#include "nav_msgs/OccupancyGrid.h"


namespace sdfslam {

    class SDFGraphMap : public SDFVectorMap {

    public:

        SDFGraphMap(){
            //rolling params different than map update params?
            window_size_ = 5;
            occ_map_pub_df_ = nh_.advertise<nav_msgs::OccupancyGrid>("local_df_map", 10);

        }

        void update_map(const PCLPointCloud& pc, const Eigen::Vector3d& pose3d){
            pos_ = pose3d;
            SDFVectorMap::update_map(pc, pose3d);
        }

        void publish_map() {
            SDFVectorMap::publish_map();

            //publish as occ df, pos +- window_size_/2 range
            double pos[2] = {pos_.x(), pos_.y()};
            util::toMap(pos, p_grid_res_, p_map_size_x_, p_map_size_y_);
            pos[0] = (int) pos[0];
            pos[1] = (int) pos[1];

            double from[2];
            from[0] = pos[0]-window_size_/2/p_grid_res_;
            from[1] = pos[1]-window_size_/2/p_grid_res_;

            double to[2];
            to[0] = pos[0]+window_size_/2/p_grid_res_;
            to[1] = pos[1]+window_size_/2/p_grid_res_;

            std::vector<int8_t> omap_df;

            for (int j = from[1]; j < to[1]; j++) {
                for (int i = from[0]; i < to[0]; i++) {
                    if (i<0 || i>p_map_size_x_ || j<0 || j>p_map_size_y_) {
                        omap_df.push_back(0); //unknown
                        omap_df.push_back(-1); //unknown
                    }
                    else {
                        int mapVal = (int) (sdf_[j][i]*100); //to cm

                        if (mapVal < -128)
                            mapVal = -128;
                        else
                            if(mapVal > 127)
                                mapVal = 127;
                        omap_df.push_back((int8_t) mapVal);
                    }
                }
            }
            nav_msgs::OccupancyGrid mapGrid;
            nav_msgs::MapMetaData metaData;

            metaData.resolution = (float) p_grid_res_;
            metaData.width = window_size_/p_grid_res_;
            metaData.height = window_size_/p_grid_res_;
            //metaData.width = window_size_/;
            //metaData.height = 50;
            metaData.origin.position.x = pos_.x()-window_size_/2;
            metaData.origin.position.y = pos_.y()-window_size_/2;
            mapGrid.data = omap_df;
            mapGrid.info = metaData;
            mapGrid.header.frame_id = p_fixed_frame_;
            occ_map_pub_df_.publish(mapGrid);


        }


    protected:

        ros::Publisher occ_map_pub_df_;
        Eigen::Vector3d pos_;
        double window_size_;



    };

}

#endif //SDF_SLAM_2D_GRAPHMAP_H
