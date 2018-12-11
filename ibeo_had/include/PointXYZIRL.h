// For disable PCL complile lib, to use PointXYZIR, and customized pointcloud    
#define PCL_NO_PRECOMPILE

//#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//#include <pcl/common/common.h>
//#include <pcl/common/centroid.h>

//Customed Point Struct for holding clustered points

/** Euclidean Velodyne coordinate, including intensity and ring number, and label. */
struct PointXYZIRL
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    uint16_t label;                     ///< point label
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;


// Register custom point struct according to PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
(uint16_t, label, label))
