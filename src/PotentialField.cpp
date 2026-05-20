#include "ouster_obstacle_avoidance/PotentialField.hpp"
#include <cmath>
#include <algorithm>

namespace ouster_obstacle_avoidance {

PotentialField::PotentialField(const Params& params)
    : params_(params)
{}

// ─────────────────────────────────────────────────────────────────────────────
//  Main entry point
// ─────────────────────────────────────────────────────────────────────────────

PushResult PotentialField::compute(const std::vector<SectorData>& sectors, const std::optional<Eigen::Vector2f>& local_goal) const
{
    PushResult result;

    Eigen::Vector2f rep_vec = computeRepulsiveVector(sectors);
    
    // Add attractive vector if goal is present
    Eigen::Vector2f att_vec = Eigen::Vector2f::Zero();
    if (local_goal.has_value()) {
        Eigen::Vector2f goal_dir = local_goal.value().normalized(); // direction to goal
        att_vec = params_.k_att * goal_dir;
    }
    
    result.push_vector       = rep_vec + att_vec;
    result.frontal_clearance = computeFrontalClearance(sectors);
    result.urgency           = computeUrgency(rep_vec); // Urgency should be based on repulsive force only!
    result.trapped           = detectTrapped(sectors, rep_vec);
    result.cmd_vel           = buildTwist(result.push_vector,
                                          result.frontal_clearance,
                                          result.urgency,
                                          result.trapped,
                                          sectors);
    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Repulsive field
// ─────────────────────────────────────────────────────────────────────────────

float PotentialField::repulsiveMagnitude(float d) const
{
    if (d >= params_.d_max || d < 1e-4f) return 0.0f;
    const float delta = (1.0f / d) - (1.0f / params_.d_max);
    return params_.k_rep * delta * delta;
}

Eigen::Vector2f PotentialField::computeRepulsiveVector(
    const std::vector<SectorData>& sectors) const
{
    Eigen::Vector2f total = Eigen::Vector2f::Zero();

    for (const auto& s : sectors)
    {
        if (!s.occupied || s.d_min >= params_.d_max) continue;

        float mag = repulsiveMagnitude(s.d_min);

        // Frontal sectors are weighted more aggressively
        float angle_wrapped = s.angle;
        if (angle_wrapped > static_cast<float>(M_PI))
            angle_wrapped -= 2.0f * static_cast<float>(M_PI);  // [-π, π]

        if (std::abs(angle_wrapped) < params_.front_half_angle)
            mag *= params_.k_front_weight;

        // Push direction: directly AWAY from obstacle sector
        const Eigen::Vector2f away(
            -std::cos(s.angle),
            -std::sin(s.angle)
        );

        total += mag * away;
    }

    return total;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Derived quantities
// ─────────────────────────────────────────────────────────────────────────────

float PotentialField::computeFrontalClearance(
    const std::vector<SectorData>& sectors) const
{
    float d_front_min = params_.d_max;

    for (const auto& s : sectors)
    {
        float angle = s.angle;
        if (angle > static_cast<float>(M_PI))
            angle -= 2.0f * static_cast<float>(M_PI);   // [-π, π]

        if (std::abs(angle) < params_.front_half_angle)
            d_front_min = std::min(d_front_min, s.d_min);
    }

    const float clearance = (d_front_min - params_.d_stop)
                          / (params_.d_warn - params_.d_stop);
    return std::clamp(clearance, 0.0f, 1.0f);
}

float PotentialField::computeUrgency(const Eigen::Vector2f& rep_vec) const
{
    // Reference magnitude: all frontal sectors at d_stop with max weight
    const float mag_at_stop  = repulsiveMagnitude(params_.d_stop)
                             * params_.k_front_weight;
    const int   front_sectors = static_cast<int>(
        params_.front_half_angle * 2.0f
        / (2.0f * static_cast<float>(M_PI) / 36.0f));   // rough count
    const float ref = mag_at_stop * static_cast<float>(std::max(front_sectors, 1));

    if (ref < 1e-6f) return 0.0f;
    return std::clamp(rep_vec.norm() / ref, 0.0f, 1.0f);
}

bool PotentialField::detectTrapped(
    const std::vector<SectorData>& sectors,
    const Eigen::Vector2f& push_vec) const
{
    int occupied_close = 0;
    for (const auto& s : sectors)
        if (s.occupied && s.d_min < params_.d_warn) ++occupied_close;

    const float ratio = static_cast<float>(occupied_close)
                      / static_cast<float>(sectors.size());

    return (ratio        > params_.trapped_sector_ratio)
        && (push_vec.norm() < params_.f_cancel_threshold);
}

int PotentialField::maxClearanceSector(
    const std::vector<SectorData>& sectors) const
{
    int   best   = 0;
    float best_d = 0.0f;
    for (int i = 0; i < static_cast<int>(sectors.size()); ++i)
    {
        if (sectors[i].d_min > best_d) {
            best_d = sectors[i].d_min;
            best   = i;
        }
    }
    return best;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Twist assembly
// ─────────────────────────────────────────────────────────────────────────────

geometry_msgs::Twist PotentialField::buildTwist(
    const Eigen::Vector2f& push_vec,
    float frontal_clearance,
    float urgency,
    bool  trapped,
    const std::vector<SectorData>& sectors) const
{
    geometry_msgs::Twist twist;

    if (trapped)
    {
        // Rotate in place toward the most open sector
        const int   escape_s     = maxClearanceSector(sectors);
        float       escape_angle = sectors[escape_s].angle;
        if (escape_angle > static_cast<float>(M_PI))
            escape_angle -= 2.0f * static_cast<float>(M_PI);  // [-π, π]

        twist.linear.x  = 0.0;
        twist.angular.z = params_.k_escape * (escape_angle >= 0.0f ? 1.0 : -1.0);
        return twist;
    }

    // Linear velocity: quadratic decay from v_max to 0 as front closes
    const float linear = params_.v_max
                       * frontal_clearance * frontal_clearance;

    // Angular: proportional to push angle, boosted by urgency
    const float push_angle    = std::atan2(push_vec.y(), push_vec.x());
    const float urgency_boost = 1.0f + params_.k_boost * urgency;
    float angular = params_.k_ang * push_angle * urgency_boost;

    // Cap angular to avoid destabilising fast turns at speed
    const float v_ratio   = (params_.v_max > 1e-4f) ? linear / params_.v_max : 0.0f;
    const float omega_cap = params_.omega_max
                          * (1.0f - params_.k_omega_speed_scale * v_ratio);
    angular = std::clamp(angular, -omega_cap, omega_cap);

    twist.linear.x  = static_cast<double>(linear);
    twist.angular.z = static_cast<double>(angular);
    return twist;
}

} // namespace ouster_obstacle_avoidance
