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

#include "SDFSlam.h"

namespace sdfslam{

    SignedDistanceField::SignedDistanceField() {
      ros::NodeHandle private_nh("~");

      private_nh.param("p_publish_map_", p_publish_map_, false);
      private_nh.param("p_fine_pos_", p_fine_pos_, true);
      private_nh.param("p_initial_pose_", p_initial_pose_, false);
      private_nh.param("p_perma_map_", p_perma_map_, false);
      private_nh.param("p_avg_mapping_", p_avg_mapping_, false);

      private_nh.param("p_grid_res_", p_grid_res_, 0.05);
      private_nh.param("p_map_size_x_", p_map_size_x_, 15);
      p_map_size_x_ /= p_grid_res_;
      private_nh.param("p_map_size_y_", p_map_size_y_, 15);
      p_map_size_y_ /= p_grid_res_;
      private_nh.param("p_stop_iter_time_", p_stop_iter_time_, 0.09);
      private_nh.param("p_num_iter_", p_num_iter_, 30);
      private_nh.param("p_min_range_", p_min_range_, 0.15);
      private_nh.param("p_max_range_", p_max_range_, 5.6);
      private_nh.param("p_timeout_ticks_", p_timeout_ticks_, 0);
      private_nh.param("p_update_map_aoe_", p_update_map_aoe_, 5);
      private_nh.param("p_yaw_threshold_", p_yaw_threshold_, 0.9);
      private_nh.param("p_dist_threshold_", p_dist_threshold_, 0.4);
      private_nh.param("p_reg_threshold_", p_reg_threshold_, 0.1);
      private_nh.param("p_avg_range_", p_avg_range_, 0);

      private_nh.param("p_vanity_", p_vanity_, 0);

      private_nh.param<std::string>("p_robot_frame_", p_robot_frame_, "/robot0");
      private_nh.param<std::string>("p_fixed_frame_", p_fixed_frame_, "/map");
      private_nh.param<std::string>("p_tar_frame_", p_tar_frame_, "/sdf_pos");
      private_nh.param<std::string>("p_scan_topic_", p_scan_topic_, "/scan");

      epsilon_ = 0.0000001;

      p_scan_rays_ = 666;
      p_scan_subscriber_queue_size_ = 100;
      time_out_counter_ = 0;
      mapping_ = true;
      map_empty_ = true;
      lastMapPos_ = Eigen::Vector3f(0, 0, 0);
      not_converged_counter_ = 0;
      pos_ = Eigen::Vector3f(0, 0, 0);
      publishMapServiceCalled_ = false;

      p_debug_ = false;
      //deprecated
      overrideMap_ = true; //false: only update if cell unknown
      pose_estimation_ = true;
      //p_scan_topic_ = "/base_scan_front";          
      p_isovalue_ = 0.00011; //threshold for object dist
      p_truncation_ = 2.5; //cap sdf map at distance p_truncation
      p_overshoot_ = (int) (0.11 / p_grid_res_); //raytrace overshoot
      //deprecated
      //p_weight_ = 1; 
      //p_interpl_radius_ = 0.20;

      // mapping threshs
      printf("res %f, map size x %d, map size y %d, overshoot %d\n", p_grid_res_, p_map_size_x_, p_map_size_y_,
             p_overshoot_);
//  img_pub_ = it_.advertise("some_img", 10);
      //matB_pub_ = it_.advertise("MatB", 10);
      //matA_pub_ = it_.advertise("MatA", 10);
      // loadMap("map.txt");
      // ROS_INFO("loadedmap");

      VecMapFloat sdf((unsigned long) p_map_size_x_, std::vector<float>((unsigned long) p_map_size_y_, 0.0));
      sdf_ = sdf;
      VecMapFloat occ_map((unsigned long) p_map_size_x_, std::vector<float>((unsigned long) p_map_size_y_, 0.0));
      occ_map_ = occ_map;

      VecMapInt sdf_thresholded((unsigned long) p_map_size_x_, std::vector<int>((unsigned long) p_map_size_y_, 0));
      sdf_thresholded_ = sdf_thresholded;
      VecMapInt sdf_count((unsigned long) p_map_size_x_, std::vector<int>((unsigned long) p_map_size_y_, 0));
      sdf_count_ = sdf_count;
      VecMapInt sdf_level((unsigned long) p_map_size_x_, std::vector<int>((unsigned long) p_map_size_y_, 0));
      sdf_level_ = sdf_level;

      scan_sub_ = nh_.subscribe(p_scan_topic_, p_scan_subscriber_queue_size_, &SignedDistanceField::scanCb, this);

      map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_map", 10);
      iso_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("iso_map", 10);
      tfpub_ = nh_.advertise<geometry_msgs::PoseStamped>("sdf_pos", 10);
      cloudA_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloudA", 10);
      cloudB_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cloudB", 10);
      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("sdf_marker_map", 10);
      occ_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("occ_map", 10);

      serviceSaveMap = nh_.advertiseService("sdf/save_map", &SignedDistanceField::saveMapService, this);
      serviceUpdateMap = nh_.advertiseService("sdf/update_map", &SignedDistanceField::updateMapService, this);
      serviceLoadMap = nh_.advertiseService("sdf/load_map", &SignedDistanceField::loadMapService, this);
      serviceReset = nh_.advertiseService("sdf/reset", &SignedDistanceField::resetService, this);
      servicePublishMap = nh_.advertiseService("sdf/publish_map", &SignedDistanceField::publishMapService, this);
    }

    bool SignedDistanceField::resetService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
      lastMapPos_ = Eigen::Vector3f(0, 0, 0);
      map_empty_ = true;
      VecMapFloat sdf((unsigned long) p_map_size_x_, std::vector<float>((unsigned long) p_map_size_y_, 0.0));
      sdf_ = sdf;
      std::vector<std::vector<int> > sdf_thresholded((unsigned long) p_map_size_x_, std::vector<int>((unsigned long) p_map_size_y_, 0));
      sdf_thresholded_ = sdf_thresholded;
      std::vector<std::vector<int> > sdf_count((unsigned long) p_map_size_x_, std::vector<int>((unsigned long) p_map_size_y_, 0));
      sdf_count_ = sdf_count;
      std::vector<std::vector<int> > sdf_level((unsigned long) p_map_size_x_, std::vector<int>((unsigned long) p_map_size_y_, 0));
      sdf_level_ = sdf_level;
      ROS_INFO("Resetting sdf..");
      return true;
    }


    void SignedDistanceField::simple_raytrace(Eigen::Vector2d src, Eigen::Vector2d tar, VecMapFloat &aMap) {

      double x0 = src.x() / p_grid_res_ + p_map_size_x_ / 2;
      double y0 = src.y() / p_grid_res_ + p_map_size_y_ / 2;
      double x1 = tar.x() / p_grid_res_ + p_map_size_x_ / 2;
      //  x1 = floor(x1);
      double y1 = tar.y() / p_grid_res_ + p_map_size_y_ / 2;
      //y1 = floor(y1);

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


    void SignedDistanceField::publishMarkerMap() {
      ROS_WARN("1");

      visualization_msgs::Marker cube_list, line_list, sphere_list;
      sphere_list.header.frame_id = cube_list.header.frame_id = line_list.header.frame_id = p_fixed_frame_;
      sphere_list.header.stamp = cube_list.header.stamp = line_list.header.stamp = ros::Time::now();
      sphere_list.ns = cube_list.ns = line_list.ns = "sdf_marker_map";
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

      //loop here
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

      marker_pub_.publish(line_list);
      marker_pub_.publish(sphere_list);
      //marker_pub_.publish(cube_list);

    }

    bool SignedDistanceField::updateMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
//bool SignedDistanceField::saveMapService(){
      updateMapServiceCalled_ = true;
      ROS_INFO("Updating map");
      return true;
    }

    bool SignedDistanceField::publishMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
      publishMapServiceCalled_ = true;
      ROS_INFO("Publish map service called");
      return true;
    }


    bool SignedDistanceField::saveMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
//bool SignedDistanceField::saveMapService(){
      saveMap("map.txt");
      ROS_INFO("Saving map");
      return true;
    }

    bool SignedDistanceField::loadMapService(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res) {
      loadMapServiceCalled_ = true;
      ROS_INFO("Loading map");
      return true;
    }


    SignedDistanceField::~SignedDistanceField() {
    }

    bool SignedDistanceField::checkTimeout() {
      if (p_timeout_ticks_ != 0) {
        if (time_out_counter_++ > p_timeout_ticks_) {
          ROS_INFO("Time out ticks %d", time_out_counter_);
          publishMarkerMap();
          return false;
        }
      }
      return true;
    }

//mapsize dubdidu
    void SignedDistanceField::loadMap(std::string filename) {
      std::string word;
      std::ifstream myfile(filename.c_str());
      if (myfile.is_open()) {
        int xC = 0;
        int yC = 0;
        while (myfile >> word) {
          //sdf_[xC][yC] = std::stof(word);
          //	  ROS_INFO("adding %f to %d %d", sdf_[xC][yC], xC, yC);
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

    void SignedDistanceField::saveMap(std::string filename) {
      std::fstream os(filename.c_str(), std::ios::out | std::ios::app);

      for (int i = 0; i < p_map_size_y_; ++i) {
        for (int j = 0; j < p_map_size_x_; ++j) {
          os << sdf_[i][j] << " ";
        }
        os << "\n";
      }

      os.close();
    }


    void SignedDistanceField::checkTodos() {
      time_out_counter_ = 0;

      if (loadMapServiceCalled_) {
        loadMap("map.txt");
        loadMapServiceCalled_ = false;
      }

      if ((map_empty_ && p_initial_pose_) || !pose_estimation_) {
        tf::StampedTransform transform;
        try {
          ros::Time now = ros::Time(0);
          tf_.waitForTransform(p_fixed_frame_, p_robot_frame_, now, ros::Duration(10.0));
          tf_.lookupTransform(p_fixed_frame_, p_robot_frame_, now, transform);
          double roll, pitch, yaw;
          tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
          ROS_INFO("External pose update: x %f y %f yaw %f", transform.getOrigin()[0], transform.getOrigin()[1], yaw);
          pos_ = Eigen::Vector3f(transform.getOrigin()[0], transform.getOrigin()[1], (yaw));
        } catch (tf::TransformException ex) {
          ROS_ERROR("No mocap data received. %s", ex.what());
        }
      }


    }

    void SignedDistanceField::scanCb(const sensor_msgs::LaserScan::ConstPtr& scan) {

      ros::Time tstart = ros::Time::now();
      checkTodos();

      sensor_msgs::PointCloud2 laser_point_cloud;
      projector_.projectLaser(*scan, laser_point_cloud);
      PCLPointCloud pc;
      pcl::fromROSMsg(laser_point_cloud, pc);

      tf::Transform sensorToWorldTf;
      tf::Quaternion rotation;
      Eigen::Matrix4f sensorToWorld;
      Eigen::Vector3f pos_delta(0, 0, 0);

      //register
      if (!map_empty_ && pose_estimation_) {
        int case_count[4];
        pos_delta = newMatch(pc, sdf_, case_count);

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

      //updatemap check
      bool update_map = map_empty_;
      // ROS_ERROR("last map pos  %f %f %f", lastMapPos_[0], lastMapPos_[1], lastMapPos_[2]);
      // ROS_ERROR("pos: %f %f %f", pos_[0], pos_[1], pos_[2]);
      // ROS_ERROR("map empty %d", map_empty_);
      if (fabs(lastMapPos_.z() - pos_.z()) > p_yaw_threshold_) {
        //    ROS_ERROR("set flag");
        update_map = true;
      }
      else if ((pos_ - lastMapPos_).norm() > p_dist_threshold_) {
        update_map = true;
        //ROS_ERROR("set flag 2");
      }

      if (update_map) map_flag_ = true;

      //transform cloud
      rotation.setRPY(0, 0, pos_.z());
      sensorToWorldTf.setRotation(rotation);
      sensorToWorldTf.setOrigin(tf::Vector3(pos_.x(), pos_.y(), 0));
      pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
      pcl::transformPointCloud(pc, pc, sensorToWorld);

      //publish tf
      sensorToWorldTf.setOrigin(tf::Vector3(pos_.x(), pos_.y(), 0));
      rotation.setRPY(0, 0, pos_.z());
      sensorToWorldTf.setRotation(rotation);
      //tfbr_.sendTransform(tf::StampedTransform( sensorToWorldTf, ros::Time::now(), p_fixed_frame_, p_tar_frame_) );
      tfbr_.sendTransform(
              tf::StampedTransform(sensorToWorldTf, scan->header.stamp, p_fixed_frame_, p_tar_frame_)); //todo !!!

      geometry_msgs::PoseStamped pose_msg;
      //pose_msg.header.stamp = scan.header.stamp;
      pose_msg.header.stamp = ros::Time::now();
      //pose_msg.header.stamp = testTime;
      //ROS_ERROR("scan stamp %d %d, time now: %d %d", scan.header.stamp.sec, scan.header.stamp.nsec, pose_msg.header.stamp.sec, pose_msg.header.stamp.nsec); 
      pose_msg.pose.position.x = pos_.x();
      pose_msg.pose.position.y = pos_.y();
      // pose_msg.pose.orientation.x = rotation.x;
      // pose_msg.pose.orientation.y = rotation.y;
      // pose_msg.pose.orientation.z = rotation.z;
      // pose_msg.pose.orientation.w = rotation.w;
      tfpub_.publish(pose_msg);

      //publish scan
      sensor_msgs::PointCloud2 cloudMsg2;
      pcl::toROSMsg(pc, cloudMsg2);
      cloudMsg2.header.frame_id = p_fixed_frame_;
      cloudMsg2.header.stamp = scan->header.stamp;
      cloudB_pub_.publish(cloudMsg2);

      if (p_vanity_ >= 3) {
        PCLPointCloud::const_iterator it = pc.begin();
        while (it < pc.end()) {
          Eigen::Vector2d a_point(it->x, it->y);
          Eigen::Vector2d cur_pos(pos_.x(), pos_.y());
          simple_raytrace(cur_pos, a_point, occ_map_);
          it++;
        }
      }
      if (p_vanity_ >= 4) {
        publishMapServiceCalled_ = true;
      }

      if (publishMapServiceCalled_) {
        publishMapServiceCalled_ = false;
        publishMarkerMap();
      }


      //  if ((map_empty_ || update_map ) && mapping_ ){
      //if ( ((map_empty_ || (map_flag_ && converged_) ) && mapping_) || updateMapServiceCalled_){
      if ((((map_flag_ && converged_) || (map_empty_)) && mapping_) || updateMapServiceCalled_) {
        //if (converged_ || map_empty_){

        if (p_vanity_ >= 1) {
          PCLPointCloud::const_iterator it = pc.begin();
          while (it < pc.end()) {
            Eigen::Vector2d a_point(it->x, it->y);
            Eigen::Vector2d cur_pos(pos_.x(), pos_.y());
            simple_raytrace(cur_pos, a_point, occ_map_);
            it++;
          }
        }
        if (p_vanity_ >= 2) {
          publishMapServiceCalled_ = true;
          //publishMarkerMap();
        }

        updateMapServiceCalled_ = false;
        sameScan = pc;
        lastMapPos_ = pos_;
        //    ROS_ERROR("last map pos @ mapping eq pos: %f %f %f", lastMapPos_[0], lastMapPos_[1], lastMapPos_[2]);
        //
        //    ROS_ERROR("mapping with pos %f %f %f", pos_.x(), pos_.y(), pos_.z());
        // sensorToWorldTf.setOrigin( tf::Vector3(pos_delta.x(), pos_delta.y(), 0) );
        // rotation.setRPY(0, 0, pos_delta.z());
        // sensorToWorldTf.setRotation(rotation);
        // pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
        // pcl::transformPointCloud(pc, pc, sensorToWorld);

        ros::Time start = ros::Time::now();
        //map(pc, sdf_);
        regression_map(pc, sdf_);
        map_flag_ = false;
        ros::Time end = ros::Time::now();
        ros::Duration dur = end - start;
        if (dur.nsec > 100000000)
          ROS_INFO("map dur         %d %d", dur.sec, dur.nsec);

        if (map_empty_) map_empty_ = false;

        start = ros::Time::now();
        publishMap(true);
        end = ros::Time::now();
        dur = end - start;
        if (dur.nsec > 100000000)
          ROS_ERROR("publish map dur %d %d", dur.sec, dur.nsec);

        //saveMap("asdmap");
      }
      else {
        ros::Time start = ros::Time::now();
        //publishMap(false);
        ros::Time end = ros::Time::now();
        ros::Duration dur = end - start;
        if (dur.nsec > 100000000)
          ROS_ERROR("publish map dur %d %d", dur.sec, dur.nsec);
      }

      ros::Time tend = ros::Time::now();
      ros::Duration tdur = tend - tstart;
      if (tdur.nsec > 100000000)
        ROS_ERROR("total dur       %d %d", tdur.sec, tdur.nsec);

    }


    void SignedDistanceField::map(PCLPointCloud &pc, VecMapFloat& aMap) {
      //for inter cell interp..
      PCLPointCloud::const_iterator it = pc.begin();
      float points[50][2];//todo magic
      float a_point_in_rl[2];
      float a_point_in_map[2];
      float point_window[2];
      float pos[2] = {pos_[0], pos_[1]};
      while (it < pc.end()) {
        a_point_in_rl[0] = it->x;
        a_point_in_rl[1] = it->y;

        a_point_in_map[0] = a_point_in_rl[0];
        a_point_in_map[1] = a_point_in_rl[1];
        (a_point_in_map);
        point_window[0] = (float) floor(a_point_in_map[0]);
        point_window[1] = (float) floor(a_point_in_map[1]);
        //      Eigen::Vector2f a_point_in_rl(it->x, it->y);
        //      Eigen::Vector2f a_point_in_map = toMap(a_point_in_rl);
        //Eigen::Vector2i point_window = a_point_in_map.cast<int>();
        //    ROS_ERROR("under consideration %f %f", a_point_in_map[0], a_point_in_map[1]);
        //    ROS_ERROR("window %f %f", point_window[0], point_window[1]);
        bool cont = true;
        int i = 0;
        while (cont) {
          //points[i] = Eigen::Vector2f(it->x, it->y);
          points[i][0] = it->x;
          points[i][1] = it->y;
          //ROS_ERROR("added %d %f %f", i, points[i][0], points[i][1]);
          i++;
          it++;
          if (it < pc.end()) {
            //    	  a_point_in_rl = Eigen::Vector2f(it->x, it->y);
            a_point_in_rl[0] = it->x;
            a_point_in_rl[1] = it->y;
            a_point_in_map[0] = a_point_in_rl[0];
            a_point_in_map[1] = a_point_in_rl[1];
            util::toMap(a_point_in_map,p_grid_res_,p_map_size_x_,p_map_size_y_);
            //Eigen::Vector2i next_point_window = a_point_in_map.cast<int>();
            float next_point_window[2] = {(float) floor(a_point_in_map[0]), (float) floor(a_point_in_map[1])};
            //ROS_ERROR("under consideration %f %f", a_point_in_map[0], a_point_in_map[1]);
            if (!(next_point_window[0] == point_window[0] && next_point_window[1] == point_window[1])) {
              cont = false;
            }
          }
          else
            cont = false;
        }
        //ROS_ERROR("src %f %f", pos_.x(), pos_.y());
        //ROS_ERROR("raytrace num %d", i);
        if (i > 1)
          raytrace(pos, points, i, sdf_);
        else {
          if ((it - 1) > pc.begin() && (it) < pc.end()) {//todo rm hackhack
            if (util::p2pDist((it - 2)->x, (it - 2)->y, points[0]) < util::p2pDist(it->x, it->y, points[0])) {
              //if ( ( (Eigen::Vector2f((it-2)->x,(it-2)->y)-points[0]).norm() )
              //   < ((Eigen::Vector2f( (it)->x, (it)->y )-points[0]).norm()) )
              //points[i] = Eigen::Vector2f((it-2)->x, (it-2)->y);
              points[i][0] = (it - 2)->x;
              points[i][1] = (it - 2)->y;
            }
            else {
              points[i][0] = it->x;
              points[i][1] = it->y;
              //	  points[i] = Eigen::Vector2f((it)->x, (it)->y);
            }
          }
          else if (it - 1 > pc.begin()) {
            //	  points[i] = Eigen::Vector2f((it-2)->x, (it-2)->y);
            points[i][0] = (it - 2)->x;
            points[i][1] = (it - 2)->y;
          }
          else if (it < pc.end()) {
            points[i][0] = it->x;
            points[i][1] = it->y;
            //points[i] = Eigen::Vector2f((it)->x, (it)->y);
          }
          raytrace(pos, points, 0, sdf_);
        }
      }

    }

    void SignedDistanceField::regression_map(PCLPointCloud &pc, VecMapFloat& aMap) {
      PCLPointCloud::const_iterator it = pc.begin();

      float points[p_scan_rays_][2]; //todo should actually calc the size using sensor range, resolution and grid size..
      float a_point_in_rl[2];
      float a_point_in_map[2];
      float point_window[2];
      float pos[2] = {pos_[0], pos_[1]};
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
            ROS_ERROR("WASTSDFADFDSAFDSFADV");
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

            //note: this is not always correct, e.g. corner case:
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
          //todo rm hackhack, look at neighbour scans (see note above..)
          //also, introduce distance threshold for neigbours, if exceeded num=1...
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

    void SignedDistanceField::deming_regression(float tars[][2], int num, double &beta0, double &beta1) {
      //todo hackhack, use neighbors
      bool revert = false;
      if (num == 0) {
        num = 2;
        revert = true;
      }

      double sumX = 0;
      double sumY = 0;
      double sumXY = 0;
      double sumX2 = 0;

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

    void SignedDistanceField::update_cells(float *src, float tars[][2], int num, float *window, VecMapFloat &aMap) {
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


    Eigen::Vector3f SignedDistanceField::newMatch(const PCLPointCloud& scan, const VecMapFloat& aMap, int* case_count) {
      ros::Time start = ros::Time::now();
      ros::Time end;
      ros::Duration dur;

      tempYaw_ = 0;
      for (int i = 0; i < 3; i++) {
        searchdirplus[i] = false;
      }

      converged_ = false;
      float searchfacs[3] = {1, 1, p_num_iter_ / 3}; //todo magic
      //float searchfacs[3] = { 1, 1, 1};
      bool interrupted = false;
      Eigen::Vector3f pose_estimate(0, 0, 0);
      Eigen::Vector3f temp_pose_estimate(0, 0, 0);
      bool breaking = true;
      for (int i = 0; i < p_num_iter_; ++i) {
        end = ros::Time::now();
        dur = end - start;
        if (dur.toSec() > p_stop_iter_time_) {
          //ROS_WARN("match dur       %d %d", dur.sec, dur.nsec);
          break;
        }

        case_count[0] = 0;
        case_count[1] = 0;
        case_count[2] = 0;
        case_count[3] = 0;
        //ROS_WARN("44 iter %d pose x %f y %f yaw %f", i, pose_estimate[0], pose_estimate[1], pose_estimate[2]);

        if (true && i < p_num_iter_ / 2)
          interrupted = estimateTransformationLogLh(pose_estimate, scan, searchfacs, aMap, case_count, false);
        else {
          estimateTransformationLogLh(pose_estimate, scan, searchfacs, aMap, case_count, p_fine_pos_);
          interrupted = false;
        }

        breaking = true;
        for (int x = 0; x < 3; x++) {
          if (fabs(temp_pose_estimate[x] - pose_estimate[x]) > 0.00001) {
            breaking = false;
          }
        }
        temp_pose_estimate = Eigen::Vector3f(pose_estimate);

        if (interrupted) {
          if (pose_estimate[0] != pose_estimate[0])
            pose_estimate = Eigen::Vector3f(0, 0, 0);
          i = p_num_iter_ / 2;
          continue;
        }

        if (breaking) {
          if (i < p_num_iter_ / 2)
            i = p_num_iter_ / 2;
          else {
            converged_ = true;
            break;
          }
        }
      }
      return pose_estimate;
    }

    bool SignedDistanceField::estimateTransformationLogLh(Eigen::Vector3f &estimate,
                                                          PCLPointCloud const &cloud,
                                                          float *searchfacs,
                                                          VecMapFloat const &aMap,
                                                          int *case_count,
                                                          bool fine) {
      //getCompleteHessianDerivs(estimate, cloud, H, dTr, aMap, false);
      //std::cout << H << std::endl << dTr << std::endl <<H.inverse() << std::endl;
      //if (true || (H(0, 0) == 0.0f) || (H(1, 1) == 0.0f)) { //todo wtf
      getCompleteHessianDerivs(estimate, cloud, H, dTr, aMap, case_count, fine);
      //std::cout << H << std::endl << dTr << std::endl <<H.inverse() << std::endl;
      //}

      // ros::Time start = ros::Time::now();

      float determinant = H.determinant();

      // ros::Time end = ros::Time::now();
      // ros::Duration dur = end-start;
      // ROS_WARN("match dur       %d %d", dur.sec, dur.nsec);



      //  if ((determinant!=0 && determinant==determinant) && (H(0, 0) != 0.0f) && (H(1, 1) != 0.0f)) {
      if ((determinant != 0) && (H(0, 0) != 0.0f) && (H(1, 1) != 0.0f)) {

        Eigen::Vector3f t_searchDir(H.inverse() * dTr);
        float searchDir[3] = {t_searchDir[0], t_searchDir[1], t_searchDir[2]};
        //float searchDir[3] = {t_searchDir[0], t_searchDir[1], 0};

        //ROS_ERROR("22 sdir %f %f %f", searchDir[0], searchDir[1], searchDir[2]);
        if (searchDir[0] != searchDir[0]) {//todo dubidu
          ROS_ERROR("det %f", H.determinant());
          ROS_ERROR("crap, todo find reason for nan");
          std::cout << H << std::endl << dTr << std::endl << H.inverse() << std::endl;
          ROS_ERROR("fine %d", fine);
          ROS_ERROR("%d %d %d %d map flag %d counter %d", case_count[0], case_count[1], case_count[2], case_count[3],
                    map_flag_, not_converged_counter_);
          exit(0);
          sleep(10000);
          return true;
        }

        //ROS_WARN("tempyaw %f", tempYaw_);
        if (tempYaw_ == 0) {
          tempYaw_ = searchDir[2];
        }
        else {
          if (tempYaw_ * searchDir[2] < 0) if (searchfacs[2] - 1 >= 1)
            searchfacs[2] = searchfacs[2] - 1;
          tempYaw_ = searchDir[2];
        }
        //    ROS_WARN("serachfacs %f %f %f", searchfacs[0],searchfacs[1],searchfacs[2]);
        // if (searchDir[2] > 0.2f) {
        //   searchDir[2] = 0.2f;
        //   std::cout << "SearchDir angle change too large\n";
        // } else if (searchDir[2] < -0.2f) {
        //   searchDir[2] = -0.2f;
        //   std::cout << "SearchDir angle change too large\n";
        // }
        // ROS_WARN("serachfacs %f %f %f", searchfacs[0],searchfacs[1],searchfacs[2]);
        // //hickhack
        // for (int j=2; j<3; j++){
        //   if (searchDir[j] > 0) {
        // 	if (!searchdirplus[j]){
        // 	  searchfacs[j] = searchfacs[j]-0.5;
        // 	  if (j==0) {
        // 	    searchfacs[1] = searchfacs[1]-0.5;;
        // 	  }
        // 	  if (j==1) {
        // 	    searchfacs[0] = searchfacs[0]-0.5;;
        // 	  }

        // 	}
        // 	searchdirplus[j] = true;
        //   }
        //   else{
        // 	  if (searchdirplus[j]){
        // 	    searchfacs[j] = searchfacs[j]-0.5;
        // 	  if (j==0) {
        // 	    searchfacs[1] = searchfacs[1]-0.5;;
        // 	  }
        // 	  if (j==1) {
        // 	    searchfacs[0] = searchfacs[0]-0.5;;
        // 	  }

        // 	  }
        // 	  searchdirplus[j] = false;
        //   }
        //   if (searchDir[j] < 1)
        //   	searchDir[j] = 1;
        // }
        // ROS_WARN("serachfacs %f %f %f", searchfacs[0],searchfacs[1],searchfacs[2]);

        //bool abort = false;
        //    if ( (searchDir[0] < 0.005 && searchDir[0] > -0.005) && (searchDir[1] < 0.005 && searchDir[1] > -0.005) && (searchDir[2] < 0.001 && searchDir[2] > -0.001) )
        //abort = true;

        //hier!
        // searchDir[0] = searchDir[0]*searchfacs[0];
        // searchDir[1] = searchDir[1]*searchfacs[1];
        // searchDir[2] = searchDir[2]*searchfacs[2];

        //magic tresh 0.0
        for (int i = 0; i < 3; i++)
          if (fabs(searchDir[i]) > 0.1) if (searchDir[i] > 0)
            searchDir[i] = 0.1;
          else
            searchDir[i] = -0.1;


        estimate[0] += searchDir[0];
        estimate[1] += searchDir[1];
        estimate[2] += searchDir[2];

        //if (abort && false)
        //return true;

      }
      else
        ROS_WARN("nothing todo :()");

      return false;
    }


    void SignedDistanceField::getCompleteHessianDerivs(const Eigen::Vector3f &pose,
                                                       PCLPointCloud const &cloud,
                                                       Eigen::Matrix3f &H,
                                                       Eigen::Vector3f &dTr,
                                                       VecMapFloat const &aMap,
                                                       int *case_count,
                                                       bool fine) {
      float yaw = pose[2] + pos_[2];
      float x = pose[0] + pos_[0];
      float y = pose[1] + pos_[1];
      float sinRot = sin(yaw);
      float cosRot = cos(yaw);

      H = Eigen::Matrix3f::Zero();
      dTr = Eigen::Vector3f::Zero();

      double t_H[3][3] = {{0, 0, 0},
                          {0, 0, 0},
                          {0, 0, 0}};
      double t_dTr[3] = {0, 0, 0};
      float transformedPointData[3] = {0, 0, 0};
      double currPoint[2];
      float rotDeriv = 0;
      float funVal = 0;

      // PCLPointCloud newCloud;
      // tf::Transform sensorToWorldTf;
      // tf::Quaternion rotation;
      // rotation.setRPY(0,0,yaw);
      // sensorToWorldTf.setRotation(rotation);
      // sensorToWorldTf.setOrigin(tf::Vector3(x,y,0));
      // Eigen::Matrix4f sensorToWorld;
      // pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
      // pcl::transformPointCloud(cloud, newCloud, sensorToWorld);

      for (PCLPointCloud::const_iterator it = cloud.begin(); it != cloud.end(); ++it) {
        //for (PCLPointCloud::const_iterator it = newCloud.begin(); it != newCloud.end(); ++it){
        currPoint[0] = it->x;
        currPoint[1] = it->y;

        //todo put this somewhere where dist is calculated anyway
        if (util::p2pDist(pose[0], pose[1], currPoint[0], currPoint[1]) < p_min_range_)
          continue;
        else if (util::p2pDist(pose[0], pose[1], currPoint[0], currPoint[1]) > p_max_range_)
          continue;

        //ROS_WARN("x %f y %f, yaw %f, point x %f point y %f", x, y, yaw, currPoint[0], currPoint[1]);
        //Eigen::Affine2d tfForState = Eigen::Translation2d(x, y) * Eigen::Rotation2Dd(yaw);

        Eigen::Affine2f tfForState = Eigen::Translation2f(x, y) * Eigen::Rotation2Df(yaw);
        Eigen::Vector2f transformedCoords(tfForState * Eigen::Vector2f(currPoint[0], currPoint[1]));

        //if (it == cloud.begin()) {
        //Eigen::Vector2f test(tfForState*Eigen::Vector2f(currPoint[0], currPoint[1]));
        //ROS_ERROR("affe %f %f",test.x(),test.y());
        //}
        //    Eigen::Vector2f transformedCoords(Eigen::Vector2f(currPoint[0], currPoint[1]));

        //todo throw out eigen..
        // float currPoint[2] = {it->x, it->y};
        // float rotation[2] = {sin(yaw), cos(yaw)...
        // float tfForState[3][3];
        // float transformedCoords[2];

        //mapValues(transformedCoords.cast<float>(), aMap, transformedPointData);
        case_count[mapValues(transformedCoords.cast<float>(), aMap, transformedPointData, fine)]++;
        //mapValues(transformedCoords.cast<float>(), aMap, transformedPointData, fine);


        //ROS_ERROR("tfpointdata: %f %f %f", transformedPointData[0],transformedPointData[1],transformedPointData[2]);

        // for (int wtf = 0; wtf <3; wtf++)
        //   if (transformedPointData[wtf] < epsilon_ && transformedPointData[wtf] > -epsilon_)
        // 	transformedPointData[wtf] = 0.0f;

        //funVal = 1.0f - transformedPointData[0];
        funVal = transformedPointData[0];
        //    rotDeriv = ((-sinRot * currPoint[0] - cosRot * currPoint[1]) * transformedPointData[1] + (cosRot * currPoint[0] - sinRot * currPoint[1]) * transformedPointData[2]);
        //currPoint[0]/=0.05;
        //currPoint[1]/=0.05;
        rotDeriv = ((-sinRot * currPoint[0] - cosRot * currPoint[1]) * transformedPointData[1] +
                    (cosRot * currPoint[0] - sinRot * currPoint[1]) * transformedPointData[2]);
        t_dTr[0] += transformedPointData[1] * funVal;
        t_dTr[1] += transformedPointData[2] * funVal;
        t_dTr[2] += rotDeriv * funVal;

        t_H[0][0] += transformedPointData[1] * transformedPointData[1];
        t_H[1][1] += transformedPointData[2] * transformedPointData[2];
        t_H[2][2] += rotDeriv * rotDeriv;

        t_H[0][1] += transformedPointData[1] * transformedPointData[2];
        //    ROS_ERROR("test %f", t_H[0][1]);
        t_H[0][2] += transformedPointData[1] * rotDeriv;
        t_H[1][2] += transformedPointData[2] * rotDeriv;
      }
      t_H[1][0] = t_H[0][1];
      t_H[2][0] = t_H[0][2];
      t_H[2][1] = t_H[1][2];

      for (int i = 0; i < 3; i++) {
        dTr(i, 0) = t_dTr[i];
        for (int j = 0; j < 3; j++)
          H(i, j) = t_H[i][j];
      }
    }

    int SignedDistanceField::mapValues(const Eigen::Vector2f &coords, VecMapFloat const &aMap, float *mpdxdy, bool fine) {
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
        if (indices[0] >= 0 && indices[0] <= p_map_size_x_ && indices[1] >= 0 && indices[1] <= p_map_size_y_)
          intensities[i] = aMap[indices[1]][indices[0]];
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


      //easy.
      // mpdxdy[0] = 1;

      // mpdxdy[1] = -((1-factors[1]) * ( intensities[0]*(1-factors[0]) + intensities[1]*factors[0]) +
      // 		 factors[1] * ( intensities[2]*(1-factors[0]) + intensities[3]*factors[0]));

      // mpdxdy[2] = -((1-factors[0]) * ( intensities[0]*(1-factors[1]) + intensities[2]*factors[1]) +
      // 		 factors[0] * ( intensities[1]*(1-factors[1]) + intensities[3]*factors[1]));



      // if (mpdxdy[1] < epsilon_ && mpdxdy[2] < epsilon_)
      //   mpdxdy[0] = 0;

      //   mpdxdy[1] = -( (intensities[2] - intensities[3]) * factors[1] + (1-factors[1]) * (intensities[0] - intensities[1]) );
      //   mpdxdy[2] = -( (intensities[2] - intensities[0]) * (1-factors[0]) + (factors[0]) * (intensities[3] - intensities[1]) );


      // ROS_ERROR("%f %f", intensities[2], intensities[3]);
      // ROS_ERROR("%f %f", intensities[0], intensities[1]);
      // ROS_ERROR("fac x %f fac y %f", factors[0], factors[1]);

      //  if (numChanges == 0)
      //ROS_ERROR("uuuhm...");


      bool allZero = true;
      for (int count = 0; count < 4; count++)
        if (intensities[count] != 0) {
          allZero = false;
        }
        else
          numChanges = 6;

      //unclean tho
      if (false || (!fine && numChanges == 0)) {
        //if (numChanges == 0){
        //follow gradient anyways todo

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
          //mpdxdy[1] = 0;
          mpdxdy[0] = factors[1] * (factors[0] * intensities[3] + (1 - factors[0]) * intensities[2])
                      + (1 - factors[1]) * (factors[0] * intensities[1] + (1 - factors[0]) * intensities[0]);
          // ROS_WARN("mp %f dx %f dy %f", mpdxdy[0], mpdxdy[1], mpdxdy[2]);
          mpdxdy[0] = fabs(mpdxdy[0]);
          //mpdxdy[0] = (0.05*0.05*mpdxdy[0])/(mpdxdy[1]+mpdxdy[2]);
          if (fabs(mpdxdy[1]) > fabs(mpdxdy[2])) {
            //mpdxdy[2] = 0;
            if (mpdxdy[1] > 0)
              mpdxdy[1] = 1;
            else
              mpdxdy[1] = -1;
          }
          else {
            //mpdxdy[1] = 0;
            if (mpdxdy[2] > 0)
              mpdxdy[2] = 1;
            else
              mpdxdy[2] = -1;
          }

          //debug
          if (mpdxdy[0] > p_grid_res_)
            mpdxdy[0] = p_grid_res_;

          //ROS_WARN("mp %f dx %f dy %f", mpdxdy[0], mpdxdy[1], mpdxdy[2]);
          // ROS_ERROR("%f %f", intensities[2], intensities[3]);
          // ROS_ERROR("%f %f", intensities[0], intensities[1]);

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

      //todo use vectors ffs
      if (false || numChanges == 2) {
        bool vertical;
        float m, b;
        float hit[2];
        float yCoord, xCoord;
        if (fabs(px[1] - px[0]) < epsilon_) { //todo schwach
          vertical = true;
          hit[0] = px[0];
          hit[1] = factors[1];
          yCoord = hit[1];
          xCoord = hit[0];
        }
        else {
          if (fabs(py[1] - py[0]) < epsilon_) { //todo schwach!!!!
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

        //mp = (sqrt(2) - sqrt( (factors[0]-xhit)*(factors[0]-xhit)
        //+ (factors[1]-yhit)*(factors[1]-yhit) ) ) /sqrt(2);

        //TODO p2ls, not euclidean, confused??
        //ROS_ERROR("m %f b %f hitx %f hity %f",m,b,hit[0],hit[1]);
        //    mpdxdy[0] = fabs(sqrt(2) - util::p2lDist(m,b,hit));
        //mpdxdy[0] = util::p2lDist(m,b,hit)/sqrt(2);
        //TODO p2lSdist!
        if (!vertical) {
          mpdxdy[0] = util::p2lDist(m, b, factors); //dunno
          //yCoord = factors[0]*m+b;
          //xCoord = (factors[1]-b)/m;
          //yCoord = hit[1];
          //xCoord = hit[0];
          yCoord = factors[0] * m + b;
          //xCoord = (factors[1]-b)/m;

          if (!m == 0)
            xCoord = (factors[1] - b) / m;
          else
            //xCoord = factors[0];
            xCoord = 999;
          //todo! not sure if this makes sense

          if (factors[0] > xCoord)
            //mpdxdy[1] = -(1-xCoord);
            mpdxdy[1] = -1;
          else
            //mpdxdy[1] = xCoord;
            mpdxdy[1] = 1;

          if (factors[1] > yCoord)
            //mpdxdy[2] = -(1-yCoord);
            mpdxdy[2] = -1;
          else
            //mpdxdy[2] = yCoord;
            mpdxdy[2] = 1;

          //mpdxdy[2] = yCoord - factors[1];
          //mpdxdy[2] = yCoord-1;

          //       //      ROS_WARN("1 mp %f dx %f dy %f", mpdxdy[0], mpdxdy[1], mpdxdy[2]);
          // mpdxdy[1] = xCoord-factors[0];
          // mpdxdy[2] = yCoord-factors[1];
          // //      ROS_WARN("2 mp %f dx %f dy %f", mpdxdy[0], mpdxdy[1], mpdxdy[2]);
          // float some_factor = (mpdxdy[1] + mpdxdy[2]);
          // mpdxdy[1] /= some_factor;
          // mpdxdy[2] /= some_factor;
          // //      ROS_WARN("3 mp %f dx %f dy %f", mpdxdy[0], mpdxdy[1], mpdxdy[2]);


          if (yCoord < 0 || yCoord > 1)
            mpdxdy[2] = 0;

          //if (mpdxdy[2] > 1)
          // if (yCoord > 1)
          // 	mpdxdy[2] = 0;
          // else
          // 	//	if (mpdxdy[2] <-1)
          // 	if (yCoord < 0)
          // 	  mpdxdy[2] = 0;

          // mpdxdy[1] = xCoord - factors[0];

          //todo same as aboce, dunno
          if (xCoord < 0 || xCoord > 1)
            mpdxdy[1] = 0;

          // else
          // 	if (xCoord < 0)
          // 	  mpdxdy[1] = 0;

          //if (xCoord < 0 || xCoord > 1)
          //mpdxdy[1] = 0;
          //      mpdxdy[0] = 0.075562;
          //      mpdxdy[2] = 0.075562;
          //      mpdxdy[1] = 0;


        }
        else {

          //todo replace all this case shit with 1 solution..
          // ROS_WARN("VERTICAL");
          if (factors[0] < px[0])
            //mpdxdy[1] = px[0];
            mpdxdy[1] = 1;
          else
            //mpdxdy[1] = px[0]-1;
            mpdxdy[1] = -1;

          mpdxdy[0] = fabs(factors[0] - px[0]);
          // //mpdxdy[1] = px[0] - factors[0];
          // mpdxdy[1] = 1-px[0];
          mpdxdy[2] = 0;

          // mpdxdy[0] = 0.075562;
          // mpdxdy[1] = 0.075562;

        }


        //ROS_WARN("mp %f dx %f dy %f", mpdxdy[0], mpdxdy[1], mpdxdy[2]);

        mpdxdy[0] = mpdxdy[0] / (1 / p_grid_res_);
        // mpdxdy[1] = mpdxdy[1]/20.0;
        // mpdxdy[2] = mpdxdy[2]/20.0;

        //ROS_WARN("div mp %f dx %f dy %f", mpdxdy[0], mpdxdy[1], mpdxdy[2]);

        mpdxdy[0] = round(mpdxdy[0], 32);
        mpdxdy[1] = round(mpdxdy[1], 32);
        mpdxdy[2] = round(mpdxdy[2], 32);

        //ROS_WARN("round mp %f dx %f dy %f", mpdxdy[0], mpdxdy[1], mpdxdy[2]);

        if (mpdxdy[1] < epsilon_ && mpdxdy[1] > -epsilon_) {
          //ROS_ERROR("asxaxaxax");
          mpdxdy[1] = 0;
        }
        if (mpdxdy[2] < epsilon_ && mpdxdy[2] > -epsilon_) {
          //ROS_ERROR("asxaxaxax");
          mpdxdy[2] = 0;
        }

        // ROS_WARN("dist %f mp %f dx %f dy %f m %f b %f", util::p2lDist(m,b,factors), mpdxdy[0], mpdxdy[1], mpdxdy[2],m,b);
        // ROS_ERROR("px0 %f py0 %f px1 %f py1 %f",px[0],py[0],px[1],py[1]);
        // ROS_ERROR("cy %f cx %f yc %f xc %f facy %f facx %f", coords[1], coords[0], yCoord, xCoord, factors[1], factors[0]);
        // ROS_ERROR("%f %f", intensities[2], intensities[3]);
        // ROS_ERROR("%f %f", intensities[0], intensities[1]);


      }

      if (numChanges == 4) {
        mpdxdy[1] = 0;
        mpdxdy[2] = 0;
        mpdxdy[0] = 0;
      }

      return numChanges / 2;
    }


    double SignedDistanceField::round(double Zahl, unsigned int Stellen) {
      Zahl *= pow(10, Stellen);
      if (Zahl >= 0)
        floor(Zahl + 0.5);
      else
        ceil(Zahl - 0.5);
      Zahl /= pow(10, Stellen);
      return Zahl;
    }

//todo easy hz here...
//todo so, theres a bug if the src||tar lies ECACTLY on the array border...
//todo update aoe around tar
    void SignedDistanceField::raytrace(float *src, float tars[][2], int num, VecMapFloat &aMap) {
      {
        float srcInMap[2] = {src[0], src[1]};
        util::toMap(srcInMap,p_grid_res_,p_map_size_x_,p_map_size_y_);
        float tarInMap[2] = {tars[0][0], tars[0][1]};
        util::toMap(tarInMap,p_grid_res_,p_map_size_x_,p_map_size_y_);

        //    ROS_ERROR("tar in map %f, %f, num %d, map val %f",tarInMap[0], tarInMap[1], num, sdf_[floor(tarInMap[1])][floor(tarInMap[0])]);

        if (!overrideMap_) {
          double mapval = sdf_[floor(tarInMap[1])][floor(tarInMap[0])];
          if (!(mapval > -epsilon_ && sdf_[floor(tarInMap[1])][floor(tarInMap[0])] < epsilon_)) {
            //	ROS_ERROR("test");
            return;
          }
        }

        //deprecated
        double mInv;
        double tmInv;
        double b;
        float tb;
        float x0, y0, x1, y1;
        double div;
        float tdiv;

        if (num == 1) {
          //deprecated
          x0 = srcInMap[0];
          y0 = srcInMap[1];
          x1 = tarInMap[0];
          y1 = tarInMap[1];
          //todo sign correct? prob..
          float m[2] = {(floor(tarInMap[0]) + 0.5 - srcInMap[0]), (floor(tarInMap[1]) + 0.5 - srcInMap[1])};
          if (m[0] == 0) {
            mInv = 0;
          }
          else {
            mInv = -1 / (m[1] / m[0]);
          }
          b = tarInMap[1] - mInv * tarInMap[0];
        }
        else {
          //todo hackhack, use neighbors
          bool revert = false;
          if (num == 0) {
            num = 2;
            revert = true;
          }

          x0 = srcInMap[0];
          y0 = srcInMap[1];
          x1 = tarInMap[0];
          y1 = tarInMap[1];

          double sumX = 0;
          double sumY = 0;
          double sumXY = 0;
          double sumX2 = 0;

          //deming new
          double xbar = 0;
          double ybar = 0;
          double sxx = 0;
          double sxy = 0;
          double syy = 0;
          for (int i = 0; i < num; i++) {
            xbar += util::toMap(tars[i][0], p_grid_res_, p_map_size_x_);
            ybar += util::toMap(tars[i][1], p_grid_res_, p_map_size_x_);
          }
          xbar /= num;
          ybar /= num;
          for (int i = 0; i < num; i++) {
            sxx += (util::toMap(tars[i][0], p_grid_res_, p_map_size_x_) - xbar) * (util::toMap(tars[i][0], p_grid_res_, p_map_size_x_) - xbar);
            sxy += (util::toMap(tars[i][0], p_grid_res_, p_map_size_x_) - xbar) * (util::toMap(tars[i][1], p_grid_res_, p_map_size_x_) - ybar);
            syy += (util::toMap(tars[i][1], p_grid_res_, p_map_size_x_) - ybar) * (util::toMap(tars[i][1], p_grid_res_, p_map_size_x_) - ybar);
          }
          sxx /= num - 1;
          sxy /= num - 1;
          syy /= num - 1;

          double beta1 = (syy - sxx + sqrt((syy - sxx) * (syy - sxx) + 4 * sxy * sxy)) / (2 * sxy);
          double beta0 = ybar - beta1 * xbar;
          //y=beta0+beta1*x

          //end deming

          bool same_x = true;
          bool same_y = true;

          for (int i = 0; i < num; i++) {
            //	if(fabs(tars[i][0]-tars[0][0])>epsilon_)
            if (fabs(tars[i][0] - tars[0][0]) > epsilon_)
              same_x = false;
            //	if(fabs(tars[i][1]-tars[0][1])>epsilon_)
            if (fabs(tars[i][1] - tars[0][1]) > epsilon_)
              same_y = false;
            sumX += util::toMap(tars[i][0], p_grid_res_, p_map_size_x_);
            sumY += util::toMap(tars[i][1], p_grid_res_, p_map_size_y_);
            sumXY += util::toMap(tars[i][0], p_grid_res_, p_map_size_x_) * util::toMap(tars[i][1], p_grid_res_, p_map_size_y_);
            sumX2 += util::toMap(tars[i][0], p_grid_res_, p_map_size_x_) * util::toMap(tars[i][0], p_grid_res_, p_map_size_x_);
          }
          div = (num * sumX2 - sumX * sumX);
          if (!same_x) {// && !(div > -0.00000001 && div < 0.00000001) ){
            mInv = (num * sumXY - sumX * sumY) / (div);
            //	ROS_WARN("m %f", mInv);
            mInv = beta1;
            //	ROS_WARN("beta1 %f", mInv);
            b = (sumY - mInv * sumX) / num;
            //	ROS_WARN("b %f", b);
            b = beta0;
            //	ROS_WARN("beta0 %f", b);
            if (same_y)
              mInv = 0;
          }
          else {
            mInv = 999999; //todo magic num
            b = 999999;
          }

          aMap[floor(y1)][floor(x1)] = 0;
          sdf_count_[floor(y1)][floor(x1)] = 0;
          //todo hack
          if (revert)
            num = 1;
        }

        for (int count = 0; count < num; count++) {

          tarInMap[0] = tars[count][0];
          tarInMap[1] = tars[count][1];
          util::toMap(tarInMap,p_grid_res_,p_map_size_x_,p_map_size_y_);

          //      ROS_ERROR("tar in map %f, %f",tarInMap[0], tarInMap[1]);

          x1 = tarInMap[0];
          y1 = tarInMap[1];

          float dx = fabs(x1 - x0);
          float dy = fabs(y1 - y0);

          int x = int(floor(x0));
          int y = int(floor(y0));

          int n = 1;
          int x_inc, y_inc;
          float error;
          float p2pDistance;
          float p2lDistance;

          if (dx == 0) {
            x_inc = 0;
            error = std::numeric_limits<float>::infinity();
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
            error -= std::numeric_limits<float>::infinity();
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


          bool invert = false;
          bool done = false;
          int overshoot = 0;
          bool invertAfter = false;

          //big fat aoe hack
          int aoehack[8][2];
          aoehack[0][0] = tarInMap[0] - 1;
          aoehack[0][1] = tarInMap[1] - 1;

          aoehack[1][0] = tarInMap[0] - 1;
          aoehack[1][1] = tarInMap[1];

          aoehack[2][0] = tarInMap[0] - 1;
          aoehack[2][1] = tarInMap[1] + 1;

          aoehack[3][0] = tarInMap[0];
          aoehack[3][1] = tarInMap[1] + 1;

          aoehack[4][0] = tarInMap[0] + 1;
          aoehack[4][1] = tarInMap[1] + 1;

          aoehack[5][0] = tarInMap[0] + 1;
          aoehack[5][1] = tarInMap[1];

          aoehack[6][0] = tarInMap[0] + 1;
          aoehack[6][1] = tarInMap[1] - 1;

          aoehack[7][0] = tarInMap[0];
          aoehack[7][1] = tarInMap[1] - 1;
          int hackcounter = 0;
          for (; n > -p_overshoot_; --n) //todo fix "rdm" overshoot
            //while (!done)
          {

            int pos[2] = {x, y};

            // if (n ==-p_overshoot_+1){
            //   x = aoehack[hackcounter][0];
            //   y = aoehack[hackcounter][1];
            //   hackcounter++;
            //   if (hackcounter < 8)
            //     n++;
            // }

            float posc[2] = {(x + 0.5), (y + 0.5)};
            float poscrl[2] = {posc[0], posc[1]};
            util::toRl(poscrl,p_grid_res_,p_map_size_x_,p_map_size_y_);

            int tart[2] = {floor(tarInMap[0]), floor(tarInMap[1])};
            float distance;

            if (false && pos[0] == tart[0] && pos[1] == tart[1]) {
              if (mInv == 999999 && b == 999999) {
                distance = fabs(poscrl[0] - tars[count][0]);
              }
              else if (mInv > -epsilon_ && mInv < epsilon_) {
                distance = fabs(poscrl[1] - tars[count][1]);
              }
              else {
                distance = util::p2lDist(mInv, b, posc);
                distance = p2lDistance * p_grid_res_;
              }
            }
            else {
              float v[2];
              float w[2];
              if (mInv == 999999 && b == 999999) {
                v[0] = tarInMap[0];
                v[1] = tart[1];
                w[0] = tarInMap[0];
                w[1] = tart[1] + 1;
              }
              else if (mInv > -epsilon_ && mInv < epsilon_) {
                v[1] = tarInMap[1];
                v[0] = tart[0];
                w[1] = tarInMap[1];
                w[0] = tart[0] + 1;
              }
              else {
                //todo, check here.. and implement small aoe update...
                //this is broken atm!!!?
                float ax, bx, cy, dy;
                ax = (tart[1] - b) / mInv;
                bx = (tart[1] + 1 - b) / mInv;
                cy = mInv * tart[0] + b;
                dy = mInv * (tart[0] + 1) + b;

                bool fp = false;
                int dbugc = 0;//todo rm
                if (ax - epsilon_ > tart[0] && ax + epsilon_ < tart[0] + 1) {
                  v[0] = ax;
                  v[1] = tart[1];
                  fp = true;
                  dbugc++;
                }
                if (bx - epsilon_ > tart[0] && bx + epsilon_ < tart[0] + 1) {
                  if (!fp) {
                    v[0] = bx;
                    v[1] = tart[1] + 1;
                    fp = true;
                    dbugc++;
                  }
                  else {
                    w[0] = bx;
                    w[1] = tart[1] + 1;
                    dbugc++;
                  }
                }
                if (cy + epsilon_ > tart[1] && cy - epsilon_ < tart[1] + 1) {
                  if (!fp) {
                    v[0] = tart[0];
                    v[1] = cy;
                    dbugc++;
                  }
                  else {
                    w[0] = tart[0];
                    w[1] = cy;
                    dbugc++;
                  }
                }
                if (dy + epsilon_ > tart[1] && dy - epsilon_ < tart[1] + 1) {
                  w[0] = tart[0] + 1;
                  w[1] = dy;
                  dbugc++;
                }
                if (dbugc > 2)
                  ROS_ERROR("FFS");
              }

              distance = util::p2lsDist(v, w, posc);
              distance *= p_grid_res_;

              float distance3 = util::p2pDist(poscrl, tars[count]);
              float distance4 = util::p2pDist(posc, tarInMap);
              distance4 *= p_grid_res_;
              if (fabs(distance3 - distance) > 1) {
                //	      ROS_ERROR("minv %f, b %f",mInv,b);
                //	      ROS_ERROR("dist p2ls %f p2p %f", distance, distance3);
                //ROS_ERROR("v %f %f w %f %f",v[0], v[1], w[0], v[1]);
                //ROS_ERROR("tart %d %d",tart[0], tart[1]);
              }
              //distance = distance3;
            }

            //distance = util::p2pDist(poscrl, tars[count]);

            if (distance < 0.00001)
              ROS_ERROR("wtf %f %f", tars[count][0], tars[count][1]);

            if (distance != distance)
              ROS_ERROR("yup, something went wrong, prob exceeded datatype acc");

            bool override = false;

            //	  if(invertAfter){
            //invert = true;
            //}

            if (pos[0] == tart[0] && pos[1] == tart[1])
              override = true;
            else {
              override = false;
            }


            if (pos[0] == tart[0] && pos[1] == tart[1]) {
              //	    ROS_ERROR("dist %f", distance);
              invert = invertCheck(srcInMap, tarInMap, posc, mInv, b);
              if (!invert) {
                invertAfter = true;
              }
            }
            else {
              //todo costly?
              invert = invertCheck(srcInMap, tarInMap, posc, mInv, b);
            }
            //old invert check
            // if (!invert && pos[0] == tart[0] && pos[1] == tart[1]){
            //   if (tarInMap[1] < tart[1]+0.5 && y1 > y0 && mInv!=999999 && b!=999999){
            //     invert = true;
            //   }
            //   else 
            //     if (tarInMap[1] > tart[1]+0.5 && y1 < y0 && mInv!=999999 && b!=999999){
            // 	invert = true;
            //     }
            //   //todo new
            //     else 
            // 	if (tarInMap[0] < tart[0]+0.5 && x1 > x0 && mInv!=999999 && b!=999999) {
            // 	  invert = true;
            // 	}
            // 	else 
            // 	  if (tarInMap[0] > tart[0]+0.5 && x1 > x0 && mInv!=999999 && b!=999999){
            // 	    invert = true;
            // 	  }
            //   //end todo
            // 	  else
            // 	    if(mInv == 999999 && b == 999999) //todo number
            // 	      if( x1 > tart[0]+0.5 && x1<x0)
            // 		invert = true;
            // 	      else
            // 		if (x1 < tart[0]+0.5 && x1>x0){
            // 		  invert = true;
            // 		}
            // 		else
            // 		  invertAfter = true;
            // 	    else
            // 	      invertAfter = true;
            // }	    


            if (invert) {
              distance *= -1;
              if (overshoot++ > p_overshoot_)
                done = true;
            }
            //if (!invert)
            //override = false;
            //override = true;
            updateMap(pos, distance, aMap, override);

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
      }
    }

    inline float SignedDistanceField::xVal(float y, float m, float b) {
      if (m != 0)
        return (y - b) / m;
      else
        ROS_ERROR("div by 0");
      return 0;
    }

    inline float SignedDistanceField::yVal(float x, float m, float b) {
      return m * x + b;
    }

    bool SignedDistanceField::invertCheck(float *src, float *tar, float *pos, float m, float b) {
      bool xVotePlus, yVotePlus;

      //fishy
      if (b == 999999 && m == b) {
        //if (fmod(tar[0],1) > 0.5)
        if (tar[0] > pos[0])
          xVotePlus = true;
        else
          xVotePlus = false;
        if (tar[0] < src[0])
          xVotePlus = !xVotePlus;
        return !xVotePlus;
      }

      if (m == 0) {
        yVotePlus = (fmod(tar[1], 1) > 0.5);
        //if (fmod(tar[1],1) >0.5)
        if (tar[1] > pos[1])
          yVotePlus = true;
        else
          yVotePlus = false;
        if (tar[1] < src[1])
          yVotePlus = !yVotePlus;
        return !yVotePlus;
      }

      float tarC[] = {floor(tar[0]) + 0.5, floor(tar[1]) + 0.5};

      float m2 = (tarC[1] - src[1]) / (tarC[0] - src[0]);

      if (m != m2) {
        float b2 = src[1] - m2 * src[0];
        float x = (b2 - b) / (m - m2);
        float y = m2 * x + b2;
        // ROS_WARN("b1 %f m1 %f b2 %f m2 %f", b, m, b2, m2);
        // ROS_WARN("x %f tarC %f src %f, tar %f", x, tarC[0], src[0], tar[0]);
        if (util::p2pDist(x, y, src) < util::p2pDist(pos, src)) {
          //if ( (m>0 && x < tarC[0] && x > src[0]) || (m<0 && x > tarC[0] && x < src[0]) ){
          // ROS_ERROR("INVERT %f smaller %f",util::p2pDist(x,y,src), util::p2pDist(src,pos));
          return true;
        }
        else {
          // ROS_ERROR("NOT INVERT %f smaller %f",util::p2pDist(x,y,src), util::p2pDist(src,pos));
          return false;
        }
      }

      //jetzt kommt quatsch
      ROS_ERROR("ding, tar x %f tar y %f", tar[0], tar[1]);
      if (fmod(tar[0], 1) > 0.5)
        xVotePlus = true;
      else
        xVotePlus = false;
      ROS_ERROR("x %d, xval %f", xVotePlus, fmod(tar[0], 1));

      if (fmod(tar[1], 1) > 0.5)
        yVotePlus = true;
      else
        yVotePlus = false;
      ROS_ERROR("y %d, yval %f", yVotePlus, fmod(tar[1], 1));

      if (fmod(xVal(src[0], m, b), 1) < 0)
        xVotePlus = !xVotePlus;
      if (fmod(yVal(src[1], m, b), 1) < 0)
        yVotePlus = !yVotePlus;
      ROS_ERROR("x %d y %d", xVotePlus, yVotePlus);


      if (m == 0)
        return yVotePlus;

      if (yVotePlus != xVotePlus) {
        ROS_ERROR("AWWWWWWWWWWWWWWWWWWWWW, m %f b %f", m, b);
        if (1 > m && m > -1)
          return !yVotePlus;
        else
          return !xVotePlus;
      }
      return !yVotePlus && !xVotePlus;
    }

    bool SignedDistanceField::invertCheck(float *src, float *tar, float border[2][2]) {
      float src_tar_dist = util::p2pDist(src, tar);
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
      float src_border_dist = util::p2pDist(sx, sy, src);

      if ((tar[0] - src[0] > 0 && sx - src[0] < 0) || (tar[0] - src[0] < 0 && sx - src[0] > 0))
        return false;
      if (src_border_dist > src_tar_dist)
        return false;
      return true;
    }


    void SignedDistanceField::updateMap(int *pos, float val, VecMapFloat &aMap, bool override) {
      int x = pos[0];
      int y = pos[1];


      if (aMap[y][x] == 0) {
        aMap[y][x] = val;
        sdf_count_[y][x] = 1;
      }
      else if ((val > 0 && aMap[y][x] > 0 && val < aMap[y][x]) || (val < 0 && aMap[y][x] < 0 && val > aMap[y][x]) ||
               override) {
        //if((val > 0 && aMap[y][x] > 0) || (val < 0 && aMap[y][x] < 0 ) || override){
        if (override) {
          {
            float temp = aMap[y][x] * sdf_count_[y][x];
            aMap[y][x] = temp + val;
            sdf_count_[y][x]++;
            aMap[y][x] = aMap[y][x] / sdf_count_[y][x];
          }
        }
        else
          aMap[y][x] = val;
      }

      if (aMap[y][x] > p_truncation_)
        aMap[y][x] = p_truncation_;
      else if (aMap[y][x] < -p_truncation_)
        aMap[y][x] = -p_truncation_;
    }


    void SignedDistanceField::publishMap(bool updated) {

      // //test


      // cv::Mat flowImage = cv::Mat(p_map_size_x_,p_map_size_x_, CV_8UC1);
      // cv_bridge::CvImage out_msg;
      // out_msg.header.stamp   = ros::Time::now(); // Same timestamp and tf frame as input image
      // out_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1; // Or whatever
      // out_msg.image    = flowImage; 
      // img_pub_.publish(out_msg.toImageMsg());



      // //end test




      if (!p_publish_map_)
        return;
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
    }
