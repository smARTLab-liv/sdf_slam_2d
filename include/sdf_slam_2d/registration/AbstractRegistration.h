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

#ifndef SDF_SLAM_2D_ABSTRACTREGISTRATION_H
#define SDF_SLAM_2D_ABSTRACTREGISTRATION_H

#include "../utility/Types.h"

namespace sdfslam{

    class AbstractRegistration {

    public:

        virtual Eigen::Vector3d new_match(const PCLPointCloud& scan, AbstractMap* const aMap, int* case_count, Eigen::Vector3d pos){};

    protected:

    };

}


#endif //SDF_SLAM_2D_ABSTRACTREGISTRATION_H
