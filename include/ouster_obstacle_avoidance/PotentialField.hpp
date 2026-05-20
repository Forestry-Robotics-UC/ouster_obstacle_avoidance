#pragma once

#include <vector>
#include <optional>
#include <Eigen/Core>

#include "ouster_obstacle_avoidance/Types.hpp"

namespace ouster_obstacle_avoidance {

class PotentialField {
public:

    struct Params {
        float d_warn              = 0.80f;
        float d_stop              = 0.25f;
        float d_max               = 1.00f;

        float k_rep               = 1.00f;
        float k_att               = 1.00f;

        float v_max               = 0.40f;
        float omega_max           = 0.80f;

        float k_ang               = 1.20f;
        float k_boost             = 0.60f;
        float k_escape            = 0.40f;

        float k_front_weight      = 1.50f;
        float front_half_angle    = 1.047f;

        float k_omega_speed_scale = 0.50f;

        float f_cancel_threshold  = 0.10f;
        float trapped_sector_ratio= 0.75f;
    };

    explicit PotentialField(const Params& params);

    PushResult compute(const std::vector<SectorData>& sectors, const std::optional<Eigen::Vector2f>& local_goal) const;

private:
    Params params_;

    float repulsiveMagnitude(float d) const;
    Eigen::Vector2f computeRepulsiveVector(const std::vector<SectorData>& sectors) const;
    float computeFrontalClearance(const std::vector<SectorData>& sectors) const;
    float computeUrgency(const Eigen::Vector2f& rep_vec) const;
    bool detectTrapped(const std::vector<SectorData>& sectors, const Eigen::Vector2f& push_vec) const;
    geometry_msgs::Twist buildTwist(const Eigen::Vector2f& push_vec,
                                    float frontal_clearance,
                                    float urgency,
                                    bool  trapped,
                                    const std::vector<SectorData>& sectors) const;
    int maxClearanceSector(const std::vector<SectorData>& sectors) const;
};

} // namespace ouster_obstacle_avoidance
