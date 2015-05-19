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

#include <ros/ros.h>
#include "SDFSlam.h"

int main(int argc, char* argv[] ) {
    ros::init(argc, argv, "sdf_2d");
    ros::NodeHandle nh("sdf_main");

    ros::NodeHandle private_nh("~");
    bool p_bag_mode;
    private_nh.param("p_bag_mode", p_bag_mode, false);

    sdfslam::SignedDistanceField sdf;

    if (!p_bag_mode)
        ros::spin();
    else {
        bool cont = true;
        unsigned int microseconds = 10000;

        while (ros::ok && cont) {
            cont = sdf.checkTimeout();
            ros::spinOnce();
            usleep(microseconds);
        }
    }
    ROS_INFO("shutting down sdf slam..");

    return 0;
}

