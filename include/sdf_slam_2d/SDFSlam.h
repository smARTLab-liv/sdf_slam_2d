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

#include "utility/Types.h"
#include "utility/UtilityFunctions.h"


#include <iostream>
#include <fstream>
#include <cmath>

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "visualization_msgs/Marker.h"
#include "nav_msgs/OccupancyGrid.h"

#include <std_srvs/Empty.h>

#include "tf_conversions/tf_eigen.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"

#include "laser_geometry/laser_geometry.h"

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>

namespace sdfslam {

    class SignedDistanceField {

    public:


        SignedDistanceField();

        ~SignedDistanceField();

        void publishMap(bool updated);

        void publishMarkerMap();

        void scanCb(const sensor_msgs::LaserScan::ConstPtr& scan);

        void raytrace(float *src, float tars[][2], int num, VecMapFloat &map);

        double round(double Zahl, unsigned int Stellen);
//
//        inline Eigen::Vector2f toMap(const Eigen::Vector2f &x);
//
//        inline Eigen::Vector2f toRl(const Eigen::Vector2f &x);
//
//        inline void toMap(float *x);
//
//        inline float toMap(const float &x, const int &s);
//
//        inline void toRl(float *x);
//
//        inline PointType genPoint(float xPos, float yPos);
//
//        inline float p2lDist(float m, float b, const Eigen::Vector2f &p);
//
//        inline float p2lDist(float m, float b, const float *p);
//
//        inline float p2pDist(float *p1, float *p2);
//
//        inline float p2pDist(float p1x, float p1y, float p2x, float p2y);
//
//        inline float p2pDist(float p1x, float p1y, float *p2);
//
//
//
//        float p2lsDist(float *v, float *w, float *p);
//
//        float p2lsDist2(float *v, float *w, float *p);

        inline float xVal(float y, float m, float b);

        inline float yVal(float x, float m, float b);

        bool checkTimeout();

        bool invertCheck(float *src, float *tar, float *pos, float m, float b);

        bool invertCheck(float *src, float *tar, float pos[2][2]);


        bool estimateTransformationLogLh(Eigen::Vector3f &estimate,
                                         PCLPointCloud const &cloud,
                                         float *searchfacs,
                                         VecMapFloat const &aMap,
                                         int *case_count,
                                         bool fine);

        Eigen::Vector3f newMatch(const PCLPointCloud &cloudIn,
                                 const VecMapFloat &aMap,
                                 int *case_count);

        void getCompleteHessianDerivs(const Eigen::Vector3f &pose,
                                      PCLPointCloud const &cloud,
                                      Eigen::Matrix3f &H,
                                      Eigen::Vector3f &dTr,
                                      VecMapFloat const &aMap,
                                      int *case_count,
                                      bool fine);

        int mapValues(const Eigen::Vector2f &coords,
                      VecMapFloat const &aMap,
                      float *mpdxdy,
                      bool fine);


        void saveMap(std::string filename);

        void loadMap(std::string filename);

        bool saveMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

        bool publishMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

        bool resetService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

        bool loadMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

        bool updateMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

        void checkTodos();


    protected:

        AbstractMap* amap;

        ros::ServiceServer serviceSaveMap;
        ros::ServiceServer serviceLoadMap;
        ros::ServiceServer serviceUpdateMap;
        ros::ServiceServer servicePublishMap;
        ros::ServiceServer serviceReset;

        int debugcounter;

        void updateMap(int *pos, float val, VecMapFloat &aMap, bool override);

        void map(PCLPointCloud &pc, VecMapFloat &aMap);

        void regression_map(PCLPointCloud &pc, VecMapFloat &aMap);

        void update_cells(float *src, float tars[][2], int num, float *window, VecMapFloat &map);

        void deming_regression(float tars[][2], int num, double &beta0, double &beta1);

        void simple_raytrace(Eigen::Vector2d src, Eigen::Vector2d tar, VecMapFloat &aMap);

        ros::NodeHandle nh_;
        ros::Subscriber scan_sub_;
        ros::Publisher map_pub_;
        ros::Publisher occ_map_pub_;
        ros::Publisher tfpub_;
        ros::Publisher iso_pub_;
        ros::Publisher cloudA_pub_;
        ros::Publisher cloudB_pub_;
        ros::Publisher marker_pub_;

        tf::TransformListener tf_;
        tf::TransformBroadcaster tfbr_;

        laser_geometry::LaserProjection projector_;

        pcl::PointCloud<pcl::PointXYZ> cloud_map_;
        pcl::PointCloud<pcl::PointXYZ> iso_map_; //rm me
        VecMapFloat sdf_;
        VecMapFloat occ_map_;
        std::vector<std::vector<int> > sdf_count_;
        std::vector<std::vector<int> > sdf_level_;
        std::vector<std::vector<int> > sdf_thresholded_;
        //  sensor_msgs::PointCloud laser_point_cloud_;

        std::string p_scan_topic_;
        std::string p_base_frame_;

        std::string p_robot_frame_;
        std::string p_tar_frame_;
        std::string p_fixed_frame_;

        Eigen::Matrix3f H;
        Eigen::Vector3f dTr;

        int p_avg_range_;
        int p_vanity_;
        double p_reg_threshold_;
        double p_min_range_;
        double p_max_range_;
        double p_stop_iter_time_;
        bool p_perma_map_;
        int p_scan_rays_;
        int p_num_iter_;
        bool p_fine_pos_;
        bool p_avg_mapping_;
        bool p_initial_pose_;
        int p_timeout_ticks_;
        int time_out_counter_;
        int not_converged_counter_;
        float epsilon_;
        int p_update_map_aoe_;
        bool p_publish_map_;
        bool map_empty_;
        bool p_debug_;
        int p_overshoot_;
        float p_truncation_;
        float p_weight_;
        uint32_t p_scan_subscriber_queue_size_;
        double p_grid_res_;
        int p_map_size_x_;
        int p_map_size_y_;
        float p_isovalue_;
        double p_yaw_threshold_;
        double p_dist_threshold_;
        Eigen::Vector3f pos_;
        Eigen::Vector3f lastMapPos_;
        PCLPointCloud sameScan;
        float p_interpl_radius_;
        bool pose_estimation_;
        bool mapping_;
        bool map_flag_;

    private:
        bool searchdirplus[3];

        //
        bool printDur_;
        bool overrideMap_;
        bool updateMapServiceCalled_;
        bool publishMapServiceCalled_;
        bool loadMapServiceCalled_;
        bool converged_;
        bool searchBoost_;
        double tempYaw_;
        double deltaXSum_;
        double deltaYSum_;
    };

}

#endif
