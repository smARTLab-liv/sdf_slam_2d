//=================================================================================================
//Copyright (C) 2015, Joscha Fossel
//
//This program is free software; you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation; either version 2 of the License, or
//any later version.
//
//This program is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without
// even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License along
//with this program; if not, write to the Free Software Foundation, Inc.,
//51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//=================================================================================================

//todo
// a) remove fishy vanity param:
// 0: no visual map maintained 1:on mapping update visual map, 2:on mapping publish visual map,
// 3:update visual map every iteration, 4:publish visual and internal map every iteration
//
// b) fix map save & load
//
// c) registration time limit variable depending on mapping/publishing dur..

#include "SDFSlam.h"


namespace sdfslam{

    SignedDistanceField::SignedDistanceField() {
        ros::NodeHandle private_nh("~");
        private_nh.param("p_publish_map_", p_publish_map_, false);
        private_nh.param("p_initial_pose_", p_initial_pose_, false);
        private_nh.param("p_perma_map_", p_perma_map_, false);
        private_nh.param("p_mapping_", p_mapping_, true);
        private_nh.param("p_grid_res_", p_grid_res_, 0.05);
        private_nh.param("p_map_size_x_", p_map_size_x_, 15);
        private_nh.param("p_map_size_y_", p_map_size_y_, 15);
        private_nh.param("p_timeout_ticks_", p_timeout_ticks_, 0);
        private_nh.param("p_yaw_threshold_", p_yaw_threshold_, 0.9);
        private_nh.param("p_dist_threshold_", p_dist_threshold_, 0.4);
        private_nh.param("p_reg_threshold_", p_reg_threshold_, 0.1);
        private_nh.param("p_vanity_", p_vanity_, 0);
        private_nh.param("p_time_warning_threshold_", p_time_warning_threshold, 0.1);
        private_nh.param("p_scan_subscriber_queue_size_", p_scan_subscriber_queue_size_, 100);
        private_nh.param<std::string>("p_robot_frame_", p_robot_frame_, "/robot0");
        private_nh.param<std::string>("p_fixed_frame_", p_fixed_frame_, "/map");
        private_nh.param<std::string>("p_tar_frame_", p_tar_frame_, "/sdf_pos");
        private_nh.param<std::string>("p_scan_topic_", p_scan_topic_, "/scan");
        p_map_size_y_ /= p_grid_res_;
        p_map_size_x_ /= p_grid_res_;
        p_time_warning_threshold *= 1000000000; //s to ns

        time_out_counter_ = 0;
        not_converged_counter_ = 0;
        lastMapPos_ = Eigen::Vector3d(0, 0, 0);
        pos_ = Eigen::Vector3d(0, 0, 0);
        map_empty_ = true;
        publishMapServiceCalled_ = false;
        pose_estimation_ = true;

        //map_ = new SDFVectorMap();
        map_ = new SDFGraphMap();
        visualization_map_ = new OccupancyGrid();
        registration_ = new GaussNewtonRegistration();

        scan_sub_ = nh_.subscribe(p_scan_topic_, (uint32_t) p_scan_subscriber_queue_size_, &SignedDistanceField::scanCb, this);
        tfpub_ = nh_.advertise<geometry_msgs::PoseStamped>("sdf_pos", 10);
        scan_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("sdf/scan", 10);
        serviceSaveMap = nh_.advertiseService("sdf/save_map", &SignedDistanceField::saveMapService, this);
        serviceUpdateMap = nh_.advertiseService("sdf/update_map", &SignedDistanceField::updateMapService, this);
        serviceLoadMap = nh_.advertiseService("sdf/load_map", &SignedDistanceField::loadMapService, this);
        serviceReset = nh_.advertiseService("sdf/reset", &SignedDistanceField::resetService, this);
        servicePublishMap = nh_.advertiseService("sdf/publish_map", &SignedDistanceField::publishMapService, this);
        serviceCreateAndPublishVisualMap = nh_.advertiseService("sdf/createAndPublishVisualMap", &SignedDistanceField::createAndPublishVisualMap, this);

        ROS_INFO("2D SDF SLAM started with res %f, map size x %f, map size y %f\n",
                 p_grid_res_, p_map_size_x_*p_grid_res_, p_map_size_y_*p_grid_res_);
    }

    SignedDistanceField::~SignedDistanceField() {
        delete registration_;
        delete map_;
        serviceSaveMap.shutdown(); //redundant
        serviceLoadMap.shutdown();
        serviceUpdateMap.shutdown();
        servicePublishMap.shutdown();
        serviceReset.shutdown();
        scan_sub_.shutdown();
    }

    bool SignedDistanceField::createAndPublishVisualMap(std_srvs::Empty::Request &req,
                                                         std_srvs::Empty::Request &res) {
        //todo :)
    }

    bool SignedDistanceField::resetService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
        lastMapPos_ = Eigen::Vector3d(0, 0, 0);
        pos_ = Eigen::Vector3d(0, 0, 0);
        map_empty_ = true;
        map_->reset_map();
        visualization_map_->reset_map();
        ROS_INFO("Service called: resetting sdf..");
        return true;
    }

    bool SignedDistanceField::updateMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
        updateMapServiceCalled_ = true;
        ROS_INFO("Service called: updating map");
        return true;
    }

    bool SignedDistanceField::publishMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
        publishMapServiceCalled_ = true;
        ROS_INFO("Service called: publishing map");
        return true;
    }

    bool SignedDistanceField::saveMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
        map_->save_map("map.txt");
        ROS_INFO("Service called: saving map");
        return true;
    }

    bool SignedDistanceField::loadMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
        loadMapServiceCalled_ = true;
        ROS_INFO("Service called: loading map");
        return true;
    }

    bool SignedDistanceField::checkTimeout() {
        if (p_timeout_ticks_ != 0) {
            if (time_out_counter_++ > p_timeout_ticks_) {
                ROS_INFO("Time out ticks %d", time_out_counter_);
                map_->publish_map();
                visualization_map_->publish_map();
                return false;
            }
        }
        return true;
    }

    void SignedDistanceField::checkTodos() {
        time_out_counter_ = 0;

        if (loadMapServiceCalled_) {
            map_->load_map("map.txt");
            loadMapServiceCalled_ = false;
        }

        if ((map_empty_ && p_initial_pose_) || !pose_estimation_) {
            tf::StampedTransform transform;
            try {
                ros::Time now = ros::Time(0);
                double roll, pitch, yaw;
                ROS_INFO("External pose update: x %f y %f yaw %f", transform.getOrigin().getX(), transform.getOrigin().getY(), yaw);
                tf_.waitForTransform(p_fixed_frame_, p_robot_frame_, now, ros::Duration(5.0));
                tf_.lookupTransform(p_fixed_frame_, p_robot_frame_, now, transform);

                tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
                ROS_INFO("External pose update: x %f y %f yaw %f", transform.getOrigin().getX(), transform.getOrigin().getY(), yaw);
                pos_ = Eigen::Vector3d((double) transform.getOrigin().getX(), (double) transform.getOrigin().getY(), yaw);
            } catch (tf::TransformException ex) {
                ROS_ERROR("No mocap data received. %s", ex.what());
            }
            //ROS_INFO("YAFSD");
        }


    }

    void SignedDistanceField::scanCb(const sensor_msgs::LaserScan::ConstPtr& scan) {
        ros::Time tStartTotal = ros::Time::now();
        checkTodos();

        sensor_msgs::PointCloud2 laser_point_cloud;
        projector_.projectLaser(*scan, laser_point_cloud);
        PCLPointCloud pc;
        pcl::fromROSMsg(laser_point_cloud, pc);

        tf::Transform sensorToWorldTf;
        tf::Quaternion rotation;
        Eigen::Matrix4f sensorToWorld;
        Eigen::Vector3d pos_delta(0, 0, 0);

        //register
        ros::Duration tDurMatch;
        if (!map_empty_ && pose_estimation_) {
            int case_count[4];

            ros::Time tStart = ros::Time::now();
            pos_delta = registration_->new_match(pc, map_, case_count, pos_);
            ros::Time tEnd = ros::Time::now();
            tDurMatch = tEnd-tStart;

            //permanent mapping trigger
            if (case_count[0] <= not_converged_counter_ * not_converged_counter_) {
                if (p_perma_map_)
                    map_flag_ = true; //avg?
                converged_ = true;
                not_converged_counter_ = 0;
            }
            else {
                converged_ = false;
                if (map_flag_ && !p_perma_map_)
                    not_converged_counter_++;
                else if (p_perma_map_)
                    not_converged_counter_++;
            }

            pos_ += pos_delta;
        }

        //pose threshold update map check
        bool update_map_trigger = map_empty_;
        if (fabs(lastMapPos_.z() - pos_.z()) > p_yaw_threshold_) {
            update_map_trigger = true;
        }
        else if ((pos_ - lastMapPos_).norm() > p_dist_threshold_) {
            update_map_trigger = true;
        }

        if (update_map_trigger) map_flag_ = true;


        //transform cloud
        rotation.setRPY(0, 0, pos_.z());
        sensorToWorldTf.setRotation(rotation);
        sensorToWorldTf.setOrigin(tf::Vector3(pos_.x(), pos_.y(), 0));

	// //todo rm me
        // if ((((map_flag_ && converged_) || (map_empty_)) && p_mapping_) || updateMapServiceCalled_) {	
	//   Eigen::Vector3d affe(0.05,0.05,0.0);
	//   affe += pos_;
	//   sensorToWorldTf.setOrigin(tf::Vector3(affe.x(), affe.y(), 0));
	// }
	// //end rm

        pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
        pcl::transformPointCloud(pc, pc, sensorToWorld);

        //publish tf
        sensorToWorldTf.setOrigin(tf::Vector3(pos_.x(), pos_.y(), 0));
        sensorToWorldTf.setRotation(rotation);
        tfbr_.sendTransform(tf::StampedTransform(sensorToWorldTf, scan->header.stamp, p_fixed_frame_, p_tar_frame_));

        //publish pose msg
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = scan->header.stamp;
        pose_msg.pose.position.x = pos_.x();
        pose_msg.pose.position.y = pos_.y();
        pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(pos_.z());
        tfpub_.publish(pose_msg);

        //publish scan
        sensor_msgs::PointCloud2 cloudMsg2;
        pcl::toROSMsg(pc, cloudMsg2);
        cloudMsg2.header.frame_id = p_fixed_frame_;
        cloudMsg2.header.stamp = scan->header.stamp;
        scan_cloud_pub_.publish(cloudMsg2);

        //publish/update visual map?
        ros::Duration tDurMappingVisual;
        if (p_vanity_ >= 3) {
            update_map(visualization_map_,pc,pos_);
        }
        if (p_vanity_ >= 4) {
            publishMapServiceCalled_ = true;
        }

        ros::Duration tDurPublishInternal;
        ros::Duration tDurPublishVisual;
        if (publishMapServiceCalled_) {
            tDurPublishInternal = publish_map(map_);
            tDurPublishVisual = publish_map(visualization_map_);
            publishMapServiceCalled_ = false;
        }

        //update map?
        ros::Duration tDurMappingInternal;
        if ((((map_flag_ && converged_) || (map_empty_)) && p_mapping_) || updateMapServiceCalled_) {
            updateMapServiceCalled_ = false;
            lastMapPos_ = pos_;
            map_flag_ = false;
            if (map_empty_)
                map_empty_ = false;

            tDurMappingInternal = update_map(map_, pc, pos_);
            if (p_vanity_ >= 1 && p_vanity_ < 3) {
                tDurMappingVisual = update_map(visualization_map_, pc, pos_);
            }
            if (p_vanity_ >= 2) {
                publishMapServiceCalled_ = true;
            }
        }

        ros::Time tEndTotal = ros::Time::now();
        ros::Duration tdur = tEndTotal - tStartTotal;
        if (tdur.nsec > p_time_warning_threshold){
            ROS_WARN("total duration       %d %d", tdur.sec, tdur.nsec);
            ROS_INFO("matching             %d %d", tDurMatch.sec, tDurMatch.nsec);
            ROS_INFO("maping internal      %d %d", tDurMappingInternal.sec, tDurMappingInternal.nsec);
            ROS_INFO("maping visual        %d %d", tDurMappingVisual.sec, tDurMappingVisual.nsec);
            ROS_INFO("publish internal     %d %d", tDurPublishInternal.sec, tDurPublishInternal.nsec);
            ROS_INFO("publish visual       %d %d", tDurPublishVisual.sec, tDurPublishVisual.nsec);
        }
    }

    ros::Duration SignedDistanceField::update_map(AbstractMap* aMap, const PCLPointCloud& pc, const Eigen::Vector3d& pose3d) {
        ros::Time tStart = ros::Time::now();
        aMap->update_map(pc, pose3d);
        ros::Time tEnd = ros::Time::now();
        return tEnd-tStart;
    }

    ros::Duration SignedDistanceField::publish_map(AbstractMap* aMap) {
        ros::Time tStart = ros::Time::now();
        aMap->publish_map();
        ros::Time tEnd = ros::Time::now();
        return tEnd-tStart;
    }
}
