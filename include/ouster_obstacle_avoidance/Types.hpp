#pragma once

#include <vector>
#include <limits>
#include <Eigen/Core>
#include <geometry_msgs/Twist.h>

namespace ouster_obstacle_avoidance {

// ─────────────────────────────────────────────
//  State machine
// ─────────────────────────────────────────────
enum class AvoidanceState {
    NOMINAL,   // no danger, not publishing cmd_vel
    AVOIDING,  // obstacle detected, publishing cmd_vel
    COASTING   // just cleared, publishing zero vel before going silent
};

// ─────────────────────────────────────────────
//  Per-point obstacle data
//  Reconstructed from range image; nearir carried
//  along for optional soft filtering.
// ─────────────────────────────────────────────
struct ObstaclePoint {
    float x, y, z;
    float horiz_dist;  // sqrt(x² + y²), in LiDAR frame
    float nearir;      // raw near-IR value from nearir_image[row,col]
    int   row, col;    // source pixel — used for nearir lookup
};

// ─────────────────────────────────────────────
//  Per-sector aggregated data
//  One entry per angular sector (360° / n_sectors)
// ─────────────────────────────────────────────
struct SectorData {
    float d_min    = std::numeric_limits<float>::max();
    float angle    = 0.0f;   // sector centre, radians, robot frame (0 = forward)
    bool  occupied = false;
};

// ─────────────────────────────────────────────
//  Output of the potential field computation
// ─────────────────────────────────────────────
struct PushResult {
    Eigen::Vector2f  push_vector        = Eigen::Vector2f::Zero();
    float            frontal_clearance  = 1.0f;   // [0,1], 1 = fully clear ahead
    float            urgency            = 0.0f;   // [0,1], normalised |push_vector|
    bool             trapped            = false;  // forces cancelled, rotate to escape
    geometry_msgs::Twist cmd_vel;
};

} // namespace ouster_obstacle_avoidance
