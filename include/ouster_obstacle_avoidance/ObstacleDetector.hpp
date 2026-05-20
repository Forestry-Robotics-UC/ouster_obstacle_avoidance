#pragma once

#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Core>

#include "ouster_obstacle_avoidance/Types.hpp"

namespace ouster_obstacle_avoidance {

/**
 * ObstacleDetector
 *
 * Reconstructs a cropped point cloud from the Ouster range + nearir images,
 * applies a Z-band filter (robot geometry, not terrain detection), and
 * aggregates per-sector minimum ranges for the potential field.
 *
 * Beam geometry (unit vectors) is precomputed once from Ouster metadata.
 * After that, every frame is a tight loop over image pixels — no heap
 * allocation, no PCL, no heavy processing.
 */
class ObstacleDetector {
public:

    struct Params {
        // Image dimensions
        int   n_rows          = 128;
        int   n_cols          = 1024;

        // Reconstruction radius — anything beyond this is ignored.
        // Within this radius, ground is not visible (LiDAR geometry),
        // so no ground filtering is needed in the detection zone.
        float max_horiz_dist  = 1.00f;   // [m]

        // Z-band filter in LiDAR frame (robot geometry constants, not terrain).
        // Removes ground returns in the visualisation zone (1-2 m) and
        // filters traversable tall grass by min height.
        float min_z_lidar     = 0.10f;   // [m] ignore below (adjust to grass height)
        float max_z_lidar     = 1.80f;   // [m] ignore above (canopy)

        // Ouster range image encoding: multiply raw uint16 by this to get metres.
        // For OS1 default: 0.001 (values in mm).
        float range_scale     = 0.001f;

        // Number of angular sectors for aggregation
        int   n_sectors       = 36;      // 10° each

        // Row band used for the aggregated visualisation scan (column-wise min).
        // These are approximate — tune after checking your mount geometry.
        int   scan_row_min    = 40;
        int   scan_row_max    = 88;

        // Optional near-IR filter: points with nearir > this threshold are
        // likely vegetation and get down-weighted in the potential field.
        // Set to a large value to disable.
        float nearir_veg_threshold = 800.0f;  // raw counts; tune per environment
        bool  use_nearir_filter    = false;   // disabled by default
    };

    explicit ObstacleDetector(const Params& params);

    /**
     * process — main per-frame entry point.
     *
     * Extracts points from point cloud and fills sector aggregations.
     * Both output containers are cleared and repopulated on each call.
     *
     * @param cloud_msg   sensor_msgs/PointCloud2
     * @param points_out  reconstructed obstacle points within max_horiz_dist
     * @param sectors_out per-sector minimum distances (size = n_sectors)
     */
    void process(const sensor_msgs::PointCloud2& cloud_msg,
                 std::vector<ObstaclePoint>& points_out,
                 std::vector<SectorData>&    sectors_out);

    /**
     * Aggregated 1D "scan" — column-wise minimum range over [scan_row_min,
     * scan_row_max]. Size = n_cols. Used only for visualisation.
     * Valid after the most recent process() call.
     */
    const std::vector<float>& getAggregatedScan() const { return aggregated_scan_; }

private:
    Params params_;

    std::vector<float> aggregated_scan_;   // n_cols entries
    int angleToSector(float angle_rad) const;
};

} // namespace ouster_obstacle_avoidance
