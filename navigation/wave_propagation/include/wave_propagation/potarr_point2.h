
#ifndef POTARR_POINT2_H_
#define POTARR_POINT2_H_

#include <pcl/ros/register_point_struct.h>

namespace wave_propagation {
    struct PotarrPoint2 {
        float x;
        float y;
        float z;
        float pot_value;
    };
}

POINT_CLOUD_REGISTER_POINT_STRUCT(
        wave_propagation::PotarrPoint2,
        (float, x, x)
        (float, y, y)
        (float, z, z)
        (float, pot_value, pot_value));

#endif

