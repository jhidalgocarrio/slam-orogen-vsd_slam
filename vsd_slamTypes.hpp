#ifndef vsd_slam_TYPES_HPP
#define vsd_slam_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */
#include <vector>

#include <boost/uuid/uuid.hpp>

#include <base/Time.hpp>
#include <base/Eigen.hpp>
#include <base/samples/RigidBodyState.hpp>

namespace visual_stereo {

    /** Visual Stereo feature **/
    struct Feature
    {
        boost::uuids::uuid index; // Indexes of the points/samples uses to compute the relative measurement
        base::Vector3d stereo_point; // 2D stereo point (u_left, u_right, v) in pixel coordinates
        base::Vector3d point_3d; // 3D point (x, y, z) estimated point
        base::Matrix3d cov_3d; // Covariance of the points/samples uses to compute the relative measurement
    };

    /** Exteroceptive Features **/
    struct ExteroFeatures
    {
        base::Time time;
        unsigned int img_idx;
        std::vector<Feature> features;
    };

    typedef boost::uuids::uuid image_uuid;
}

#endif

