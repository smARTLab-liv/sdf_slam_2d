//=================================================================================================
//Copyright (C) 2015, Joscha Fossel
//
//Gauss Newton minimization registration extended from Hector SLAM:
//https://github.com/tu-darmstadt-ros-pkg/hector_slam
//Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
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

#ifndef SDF_SLAM_2D_GAUSSNEWTON_H
#define SDF_SLAM_2D_GAUSSNEWTON_H

#include "ros/ros.h"


namespace sdfslam{

    class GaussNewtonRegistration : public AbstractRegistration{

    public:

        GaussNewtonRegistration(){
            ros::NodeHandle private_nh("~");
            private_nh.param("p_min_range_", p_min_range_, 0.15);
            private_nh.param("p_max_range_", p_max_range_, 5.6);
            private_nh.param("p_fine_pos_", p_fine_pos_, true);
            private_nh.param("p_stop_iter_time_", p_stop_iter_time_, 0.09);
            private_nh.param("p_num_iter_", p_num_iter_, 30);
        }

        Eigen::Vector3d new_match(const PCLPointCloud& scan, AbstractMap* const aMap, int* case_count, const Eigen::Vector3d pos){
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
            Eigen::Vector3d pose_estimate(0, 0, 0);
            Eigen::Vector3d temp_pose_estimate(0, 0, 0);
            bool breaking = true;
            for (int i = 0; i < p_num_iter_; ++i) {
                end = ros::Time::now();
                dur = end - start;
                if (dur.toSec() > p_stop_iter_time_) {
                    ROS_WARN("match dur       %d %d, stopping at iter %d", dur.sec, dur.nsec, i);
                    break;
                }

                case_count[0] = 0;
                case_count[1] = 0;
                case_count[2] = 0;
                case_count[3] = 0;
                //ROS_WARN("44 iter %d pose x %f y %f yaw %f", i, pose_estimate[0], pose_estimate[1], pose_estimate[2]);

                if (true && i < p_num_iter_ / 2)
                    interrupted = estimateTransformationLogLh(pose_estimate, scan, searchfacs, aMap, case_count, false, pos);
                else {
                    estimateTransformationLogLh(pose_estimate, scan, searchfacs, aMap, case_count, p_fine_pos_, pos);
                    interrupted = false;
                }

                breaking = true;
                for (int x = 0; x < 3; x++) {
                    if (fabs(temp_pose_estimate[x] - pose_estimate[x]) > 0.00001) {
                        breaking = false;
                    }
                }
                temp_pose_estimate = Eigen::Vector3d(pose_estimate);

                if (interrupted) {
                    if (pose_estimate[0] != pose_estimate[0])
                        pose_estimate = Eigen::Vector3d(0, 0, 0);
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

    protected:
        bool estimateTransformationLogLh(Eigen::Vector3d &estimate,
                                         const PCLPointCloud &cloud,
                                         float *searchfacs,
                                         AbstractMap* const aMap,
                                         int *case_count,
                                         bool fine,
                                         Eigen::Vector3d pos) {
            //getCompleteHessianDerivs(estimate, cloud, H, dTr, aMap, false);
            //std::cout << H << std::endl << dTr << std::endl <<H.inverse() << std::endl;
            //if (true || (H(0, 0) == 0.0f) || (H(1, 1) == 0.0f)) { //todo wtf
            getCompleteHessianDerivs(estimate, cloud, H, dTr, aMap, case_count, fine, pos);
            //std::cout << H << std::endl << dTr << std::endl <<H.inverse() << std::endl;
            //}

            // ros::Time start = ros::Time::now();

            float determinant = H.determinant();

            // ros::Time end = ros::Time::now();
            // ros::Duration dur = end-start;
            // ROS_WARN("match dur       %d %d", dur.sec, dur.nsec);



            //  if ((determinant!=0 && determinant==determinant) && (H(0, 0) != 0.0f) && (H(1, 1) != 0.0f)) {
            if ((determinant != 0) && (H(0, 0) != 0.0f) && (H(1, 1) != 0.0f)) {

                Eigen::Vector3d t_searchDir(H.inverse() * dTr);
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
        void getCompleteHessianDerivs(const Eigen::Vector3d &pose,
                                      const PCLPointCloud &cloud,
                                      Eigen::Matrix3d &H,
                                      Eigen::Vector3d &dTr,
                                      AbstractMap* const aMap,
                                      int *case_count,
                                      bool fine,
                                      Eigen::Vector3d pos) {
            float yaw = pose[2] + pos[2];
            float x = pose[0] + pos[0];
            float y = pose[1] + pos[1];
            float sinRot = sin(yaw);
            float cosRot = cos(yaw);

            H = Eigen::Matrix3d::Zero();
            dTr = Eigen::Vector3d::Zero();

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

                Eigen::Affine2d tfForState = Eigen::Translation2d(x, y) * Eigen::Rotation2Dd(yaw);
                Eigen::Vector2d transformedCoords(tfForState * Eigen::Vector2d(currPoint[0], currPoint[1]));

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
                //case_count[mapValues(transformedCoords.cast<float>(), aMap, transformedPointData, fine)]++;
                case_count[aMap->get_map_values(transformedCoords.cast<double>(), transformedPointData, fine)]++;

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

        //look into those
        double tempYaw_;
        bool converged_;
        bool searchdirplus[3];
        Eigen::Matrix3d H;
        Eigen::Vector3d dTr;
        bool map_flag_;
        int not_converged_counter_;


        //these are fine
        int p_num_iter_;
        double p_stop_iter_time_;
        bool p_fine_pos_;
        double p_min_range_;
        double p_max_range_;

    };
}

#endif //SDF_SLAM_2D_GAUSSNEWTON_H
