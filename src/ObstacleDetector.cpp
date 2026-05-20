#include "ouster_obstacle_avoidance/ObstacleDetector.hpp"
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <cmath>

namespace ouster_obstacle_avoidance {

ObstacleDetector::ObstacleDetector(const Params& params)
    : params_(params)
{
    aggregated_scan_.resize(params_.n_cols, 100.0f);
}

void ObstacleDetector::process(const sensor_msgs::PointCloud2& cloud_msg,
                               std::vector<ObstaclePoint>& points_out,
                               std::vector<SectorData>&    sectors_out)
{
    points_out.clear();
    points_out.reserve(cloud_msg.width * cloud_msg.height / 10);
    
    sectors_out.assign(params_.n_sectors, {100.0f});

    const float two_pi       = 2.0f * static_cast<float>(M_PI);
    const float sector_width = two_pi / static_cast<float>(params_.n_sectors);

    // Sector centre angles
    for (int s = 0; s < params_.n_sectors; ++s)
        sectors_out[s].angle = (s + 0.5f) * sector_width;


    constexpr float kFarValue = 100.0f;
    std::fill(aggregated_scan_.begin(), aggregated_scan_.end(), kFarValue);

    const float max_horiz_sq = params_.max_horiz_dist * params_.max_horiz_dist;

    sensor_msgs::PointCloud2ConstIterator<float>    iter_x(cloud_msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float>    iter_y(cloud_msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float>    iter_z(cloud_msg, "z");
    
    // Check if we have ambient for near-ir filtering
    bool has_ambient = false;
    for (const auto& field : cloud_msg.fields) {
        if (field.name == "ambient") {
            has_ambient = true;
            break;
        }
    }
    
    // Ouster ambient is typically 16 bit
    sensor_msgs::PointCloud2ConstIterator<uint16_t> iter_amb(cloud_msg, "ambient"); 

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
        const float x = *iter_x;
        const float y = *iter_y;
        const float z = *iter_z;
        
        float nearir_val = 0;
        if (has_ambient && iter_amb != iter_amb.end()) {
            nearir_val = static_cast<float>(*iter_amb);
            ++iter_amb;
        }

        if (std::isnan(x) || (x == 0.0f && y == 0.0f && z == 0.0f)) continue;

        // ── Z-band filter (robot geometry constants, not terrain) ─────────
        if (z < params_.min_z_lidar || z > params_.max_z_lidar) continue;

        // ── Horizontal radius filter ─────────────────────────────────────
        const float horiz_sq = x*x + y*y;
        if (horiz_sq > max_horiz_sq) continue;
        const float horiz_dist = std::sqrt(horiz_sq);
        
        // ── Aggregated visualisation scan (simple azimuth mapping) ──
        const float az = std::atan2(y, x);
        const float az_norm = az < 0.0f ? az + 2.0f * M_PI : az;
        int col = static_cast<int>((az_norm / (2.0f * M_PI)) * params_.n_cols);
        col = std::clamp(col, 0, params_.n_cols - 1);
        
        if (horiz_dist < aggregated_scan_[col]) {
            aggregated_scan_[col] = horiz_dist;
        }

        // ── Optional near-IR vegetation filter ──────────────────────────
        // Only if ambient field exists and filtering is enabled
        if (params_.use_nearir_filter && has_ambient && nearir_val > params_.nearir_veg_threshold) {
             continue;
        }

        // ── Store point ──────────────────────────────────────────────────
        ObstaclePoint pt;
        pt.x = x;  pt.y = y;  pt.z = z;
        pt.horiz_dist = horiz_dist;
        pt.nearir  = nearir_val;
        points_out.push_back(pt);

        // ── Sector aggregation ───────────────────────────────────────────
        const int sector_idx = angleToSector(az_norm);
        if (horiz_dist < sectors_out[sector_idx].d_min) {
            sectors_out[sector_idx].d_min = horiz_dist;
            sectors_out[sector_idx].occupied = true;
        }
    }
}

int ObstacleDetector::angleToSector(float angle_rad) const
{
    const float sector_width = 2.0f * static_cast<float>(M_PI)
                             / static_cast<float>(params_.n_sectors);
    const int s = static_cast<int>(angle_rad / sector_width);
    return std::clamp(s, 0, params_.n_sectors - 1);
}

} // namespace ouster_obstacle_avoidance
