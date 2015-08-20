#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace sdfslam {
    typedef std::vector<std::vector<float> > VecMapFloat;
    typedef std::vector<std::vector<int> > VecMapInt;
    typedef pcl::PointXYZ PointType;
    typedef pcl::PointCloud<PointType> PCLPointCloud
    ;
}