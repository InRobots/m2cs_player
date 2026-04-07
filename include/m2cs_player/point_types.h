#ifndef POINT_TYPES_H
#define POINT_TYPES_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// for Ouster LiDAR
struct OusterPoint {
    PCL_ADD_POINT4D;
    float intensity;        // equivalent to signal
    uint32_t t;
    uint16_t reflectivity;
    uint16_t ring;          // equivalent to channel
    uint16_t ambient;       // equivalent to near_ir
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    OusterPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint16_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

// for Livox LiDAR
struct LivoxPoint {
    PCL_ADD_POINT4D;
    uint8_t reflectivity;
    uint8_t line;
    uint8_t tag;
    uint32_t offset_time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(
    LivoxPoint,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (std::uint8_t, reflectivity, reflectivity)
    (std::uint8_t, line, line)
    (std::uint8_t, tag, tag)
    (std::uint32_t, offset_time, offset_time)
)

#endif