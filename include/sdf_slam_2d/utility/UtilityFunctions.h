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

#ifndef SDF_SLAM_2D_UTILITYFUNCTIONS_H
#define SDF_SLAM_2D_UTILITYFUNCTIONS_H

#include "Types.h"
#include <cmath>
#include <Eigen/Geometry>
#include "ros/ros.h"

namespace util{

    static pcl::PointXYZ genPoint(double xPos, double yPos){
        sdfslam::PointType point;
        point.x = (float) xPos;
        point.y = (float) yPos;
        point.z = 0;
        return point;
    }

    static double round(double Zahl, unsigned int Stellen) {
        Zahl *= pow(10, Stellen);
        if (Zahl >= 0)
            floor(Zahl + 0.5);
        else
            ceil(Zahl - 0.5);
        Zahl /= pow(10, Stellen);
        return Zahl;
    }

    inline float xVal(float y, float m, float b) {
        if (m != 0)
            return (y - b) / m;
        else
            ROS_ERROR("div by 0");
        return 0;
    }

    inline float yVal(float x, float m, float b) {
        return m * x + b;
    }

    inline float p2pDist(float p1x, float p1y, float* p2){
        return sqrt( (p1x-p2[0])*(p1x-p2[0])+(p1y-p2[1])*(p1y-p2[1]) );
    }

    inline float p2pDist(float* p1, float* p2){
        return sqrt( (p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]) );
    }

    inline float p2pDist(float p1x, float p1y, float p2x, float p2y){
        return sqrt( (p1x-p2x)*(p1x-p2x)+(p1y-p2y)*(p1y-p2y) );
    }

    inline float p2lDist(float m, float b, const Eigen::Vector2f& p1){
        return ( fabs(p1.y() - m*p1.x() - b)/sqrt(m*m+1) );
    }

    inline float p2lDist(float m, float b, const float* p1){
        return ( fabs(p1[1] - m*p1[0] - b)/sqrt(m*m+1) );
    }

    inline Eigen::Vector2f toMap(double p_grid_res_, int p_map_size_x_, int p_map_size_y_, const Eigen::Vector2f& x){
        return ( x/p_grid_res_ + Eigen::Vector2f(p_map_size_x_/2, p_map_size_y_/2) );
    }

    inline float toMap(const float& x, const double& p_grid_res_, const int& s){
        return (x/p_grid_res_+s/2);
    }

    inline void toMap(float* x, const double& p_grid_res, const int& p_map_size_x, const int& p_map_size_y){
        x[0] = x[0]/p_grid_res + p_map_size_x/2;
        x[1] = x[1]/p_grid_res + p_map_size_y/2;
    }

    inline void toMap(double* x, const double& p_grid_res, const int& p_map_size_x, const int& p_map_size_y){
        x[0] = x[0]/p_grid_res + p_map_size_x/2;
        x[1] = x[1]/p_grid_res + p_map_size_y/2;
    }


    inline Eigen::Vector2f toRl(const Eigen::Vector2f& x, double p_grid_res, int p_map_size_x, int p_map_size_y){
        return ( (x-Eigen::Vector2f(p_map_size_x/2, p_map_size_y/2) ) * p_grid_res);
    }


    inline void toRl(double* x, double p_grid_res, int p_map_size_x, int p_map_size_y){
        x[0] = (x[0]-p_map_size_x/2) * p_grid_res;
        x[1] = (x[1]-p_map_size_y/2) * p_grid_res;
    }

    inline void toRl(float* x, double p_grid_res, int p_map_size_x, int p_map_size_y){
        x[0] = (float) ((x[0]-p_map_size_x/2) * p_grid_res);
        x[1] = (float) ((x[1]-p_map_size_y/2) * p_grid_res);
    }

    //return 0 if orthogonal projection not in cell
    static float p2lsDistTwo(double p_reg_threshold_, float *v, float *w, float *p) {
        // Return minimum distance between line segment vw and point p
        const float l2 = (w[0] - v[0]) * (w[0] - v[0]) + (w[1] - v[1]) * (w[1] - v[1]);  // i.e. |w-v|^2 -  avoid a sqrt
        if (l2 == 0.0) return p2pDist(p, v);   // v == w case
        // Consider the line extending the segment, parameterized as v + t (w - v).
        // We find projection of point p onto the line.
        // It falls where t = [(p-v) . (w-v)] / |w-v|^2
        const float t = ((p[0] - v[0]) * (w[0] - v[0]) + (p[1] - v[1]) * (w[1] - v[1])) / l2;
        if (t <= 0.0 - p_reg_threshold_) return 0.0;       // Beyond the 'v' end of the segment
        else if (t >= 1.0 + p_reg_threshold_) return 0.0;  // Beyond the 'w' end of the segment
        float projection[2] = {(v[0] + t * (w[0] - v[0])),
                               (v[1] + t * (w[1] - v[1]))};  // Projection falls on the segment
        return p2pDist(p, projection);
    }

    static float p2lsDist(float* v, float* w, float* p){
        // Return minimum distance between line segment vw and point p
        const float l2 = (w[0]-v[0])*(w[0]-v[0]) + (w[1]-v[1])*(w[1]-v[1]);  // i.e. |w-v|^2 -  avoid a sqrt
        if (l2 == 0.0) return p2pDist(p, v);   // v == w case
        // Consider the line extending the segment, parameterized as v + t (w - v).
        // We find projection of point p onto the line.
        // It falls where t = [(p-v) . (w-v)] / |w-v|^2
        const float t = ( (p[0] - v[0]) * (w[0] - v[0]) + (p[1] - v[1]) * (w[1] - v[1]) ) / l2;
        if (t < 0.0) return p2pDist(p, v);       // Beyond the 'v' end of the segment
        else if (t > 1.0) return p2pDist(p, w);  // Beyond the 'w' end of the segment
        float projection[2] = {  (v[0] + t * (w[0] - v[0]) ),  (v[1] + t * (w[1] - v[1])) };  // Projection falls on the segment
        return p2pDist(p, projection);
    }

}

#endif //SDF_SLAM_2D_UTILITYFUNCTIONS_H
