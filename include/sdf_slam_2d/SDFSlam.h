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


#ifndef SDF_H__
#define SDF_H__

#include "map/AbstractMap.h"
#include "map/VectorMap.h"
#include "map/OccupancyGrid.h"
#include "map/GraphMap.h"

#include "registration/AbstractRegistration.h"
#include "registration/GaussNewton.h"

#include "utility/Types.h"
#include "utility/UtilityFunctions.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

namespace sdfslam {

    class SignedDistanceField {

    public:

        SignedDistanceField();

        ~SignedDistanceField();

        bool checkTimeout();

    protected:
        void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan);

        bool saveMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

        bool publishMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

        bool resetService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

        bool loadMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

        bool updateMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

        bool createAndPublishVisualMap(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

        void checkTodos();

        ros::Duration update_map(AbstractMap* aMap, const PCLPointCloud& pc, const Eigen::Vector3d& pose3d);
        ros::Duration publish_map(AbstractMap* aMap);

        AbstractMap* map_;
        AbstractMap* visualization_map_;

        AbstractRegistration* registration_;

        ros::NodeHandle nh_;

        ros::ServiceServer serviceSaveMap;
        ros::ServiceServer serviceLoadMap;
        ros::ServiceServer serviceUpdateMap;
        ros::ServiceServer serviceCreateAndPublishVisualMap;
        ros::ServiceServer servicePublishMap;
        ros::ServiceServer serviceReset;

        ros::Subscriber scan_sub_;
        ros::Publisher tfpub_;
        ros::Publisher scan_cloud_pub_;

        tf::TransformListener tf_;
        tf::TransformBroadcaster tfbr_;

        laser_geometry::LaserProjection projector_;

        std::string p_scan_topic_;
        std::string p_robot_frame_;
        std::string p_tar_frame_;
        std::string p_fixed_frame_;
        Eigen::Vector3d pos_;
        Eigen::Vector3d lastMapPos_;
        int p_vanity_;
        int p_timeout_ticks_;
        int time_out_counter_;
        int not_converged_counter_;
        int p_map_size_x_;
        int p_map_size_y_;
        int p_scan_subscriber_queue_size_;
        double p_time_warning_threshold;
        double p_reg_threshold_;
        double p_yaw_threshold_;
        double p_dist_threshold_;
        double p_grid_res_;
        bool p_perma_map_;
        bool p_initial_pose_;
        bool p_publish_map_;
        bool map_empty_;
        bool pose_estimation_;
        bool p_mapping_;
        bool map_flag_;
        bool updateMapServiceCalled_;
        bool publishMapServiceCalled_;
        bool loadMapServiceCalled_;
        bool converged_;

    private:

    };

}

#endif



