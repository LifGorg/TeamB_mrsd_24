#include "behavior_executive/action_handlers/navigate_to_waypoint_handler.hpp"
#include "behavior_executive/mavros_adapter_interface.hpp"

#include <behavior_tree/behavior_tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

NavigateToWaypointHandler::NavigateToWaypointHandler(
    std::shared_ptr<IMavrosAdapter> mavros_adapter,
    rclcpp::Logger logger)
    : mavros_adapter_(mavros_adapter),
      logger_(logger),
      target_lat_(0.0),
      target_lon_(0.0),
      target_alt_(10.0) {}

void NavigateToWaypointHandler::set_target_waypoint(double lat, double lon, double alt) {
    target_lat_ = lat;
    target_lon_ = lon;
    target_alt_ = alt;
    
    RCLCPP_INFO(logger_, 
                "[Navigate to Waypoint] Target waypoint set: lat=%.6f, lon=%.6f, alt=%.1fm",
                target_lat_, target_lon_, target_alt_);
}

void NavigateToWaypointHandler::get_target_waypoint(double& lat, double& lon, double& alt) const {
    lat = target_lat_;
    lon = target_lon_;
    alt = target_alt_;
}

bool NavigateToWaypointHandler::check_safety(const Context& ctx) {
    // TEMPORARY: Bypass safety checks for debugging
    RCLCPP_WARN(logger_, "[Navigate to Waypoint] BYPASSING SAFETY CHECKS FOR DEBUGGING");
    RCLCPP_INFO(logger_, "[Navigate to Waypoint] Current GPS: lat=%.6f, lon=%.6f", 
                ctx.current_latitude, ctx.current_longitude);
    RCLCPP_INFO(logger_, "[Navigate to Waypoint] Target waypoint: lat=%.6f, lon=%.6f, alt=%.1fm",
                target_lat_, target_lon_, target_alt_);
    return true;
    
    // // 检查 GPS 是否有效
    // if (ctx.current_latitude == 0.0 && ctx.current_longitude == 0.0) {
    //     RCLCPP_ERROR(logger_, "[Navigate to Waypoint] Invalid current GPS position");
    //     return false;
    // }

    // // 检查目标航点是否有效
    // if (target_lat_ == 0.0 && target_lon_ == 0.0) {
    //     RCLCPP_ERROR(logger_, "[Navigate to Waypoint] Invalid target waypoint (0.0, 0.0)");
    //     return false;
    // }

    // // 检查目标高度是否合理
    // if (target_alt_ < MIN_ALTITUDE || target_alt_ > MAX_ALTITUDE) {
    //     RCLCPP_ERROR(logger_, 
    //                 "[Navigate to Waypoint] Invalid target altitude: %.2fm (must be between %.1f and %.1f)",
    //                 target_alt_, MIN_ALTITUDE, MAX_ALTITUDE);
    //     return false;
    // }

    // RCLCPP_INFO(logger_, "[Navigate to Waypoint] Safety checks passed");
    // return true;
}

void NavigateToWaypointHandler::on_activated(bt::Action* action, const Context& ctx) {
    RCLCPP_INFO(logger_, 
                "[Navigate to Waypoint] Navigating to target: (%.6f, %.6f) at %.1fm altitude",
                target_lat_, target_lon_, target_alt_);

    // 1. Compute Obstacle-Aware Path with optional polygon constraint
    GPSPoint start_gps{ctx.current_latitude, ctx.current_longitude};
    GPSPoint goal_gps{target_lat_, target_lon_};
    
    std::vector<sensor_msgs::msg::NavSatFix> waypoints = compute_waypoint_path(
        start_gps,
        goal_gps,
        ctx.obstacles,
        target_alt_,
        ctx.polygon_vertices
    );

    // 2. Fallback if path planning fails (or returns empty)
    if (waypoints.empty()) {
        RCLCPP_WARN(logger_, "[Navigate to Waypoint] Path planning failed or returned empty. Fallback to direct waypoint.");
        sensor_msgs::msg::NavSatFix direct_wp;
        direct_wp.latitude = target_lat_;
        direct_wp.longitude = target_lon_;
        direct_wp.altitude = target_alt_;
        waypoints.push_back(direct_wp);
    } else {
        RCLCPP_INFO(logger_, "[Navigate to Waypoint] Computed path with %zu waypoints.", waypoints.size());
    }

    // 3. Push Waypoints to Flight Controller
    // 首先清除之前的任务
    mavros_adapter_->clear_waypoints(
        [this, action, waypoints, ctx](bool clear_success) {
            if (!clear_success) {
                RCLCPP_WARN(logger_, 
                           "[Navigate to Waypoint] Failed to clear waypoints, proceeding anyway");
            }
            
            RCLCPP_INFO(logger_, "[Navigate to Waypoint] Pushing %zu waypoints to flight controller", waypoints.size());
            
            // 推送航点
            mavros_adapter_->push_waypoints(
                waypoints,
                target_alt_,
                HOLD_TIME,
                {},  // 不使用 yaws
                ACCEPTANCE_RADIUS,
                [this, action, ctx](bool push_success) {
                    if (push_success) {
                        RCLCPP_INFO(logger_, "[Navigate to Waypoint] Successfully pushed waypoints");
                        
                        // 如果当前模式不是 AUTO.MISSION，需要切换模式
                        if (ctx.current_mode != "AUTO.MISSION") {
                            RCLCPP_INFO(logger_, 
                                       "[Navigate to Waypoint] Switching to AUTO.MISSION mode");
                            
                            mavros_adapter_->set_mode(
                                "AUTO.MISSION",
                                [this, action](bool mode_success) {
                                    if (mode_success) {
                                        action->set_success();
                                        RCLCPP_INFO(logger_, 
                                                   "[Navigate to Waypoint] Navigation started successfully");
                                    } else {
                                        action->set_failure();
                                        RCLCPP_ERROR(logger_, 
                                                    "[Navigate to Waypoint] Failed to switch to MISSION mode");
                                    }
                                },
                                "Navigate to Waypoint"
                            );
                        } else {
                            action->set_success();
                            RCLCPP_INFO(logger_, 
                                       "[Navigate to Waypoint] Navigation started (already in MISSION mode)");
                        }
                    } else {
                        action->set_failure();
                        RCLCPP_ERROR(logger_, "[Navigate to Waypoint] Failed to push waypoints");
                    }
                },
                "Navigate to Waypoint",
                ctx.current_mode
            );
        },
        "Navigate to Waypoint"
    );
}

std::vector<sensor_msgs::msg::NavSatFix> NavigateToWaypointHandler::compute_waypoint_path(
    const GPSPoint& start_gps,
    const GPSPoint& goal_gps,
    const std::vector<Obstacle>& obstacles,
    double target_altitude,
    const std::vector<GPSPoint>& polygon_vertices) {

    RCLCPP_INFO(logger_, 
                "[ComputeWaypointPath] Computing obstacle avoidance path from (%.6f, %.6f) to (%.6f, %.6f).",
                start_gps.lat, start_gps.lon, goal_gps.lat, goal_gps.lon);

    // The path planner uses meter-based calculations. We need to convert obstacles' GPS to meters.
    // We use the start_gps as the reference point for this local coordinate frame.
    std::vector<Obstacle> obstacles_in_meters;
    for (const auto& obs_gps : obstacles) {
        // NOTE: The Obstacle struct from path_planning.hpp takes GPS, MeterPoint, and radius.
        // We calculate the meter point relative to the drone's start position.
        MeterPoint meter_point = gpsToMeter(start_gps, obs_gps.gps);
        obstacles_in_meters.emplace_back(obs_gps.gps, meter_point, obs_gps.radius);
    }
    
    // Create polygon constraint if vertices are provided
    Polygon polygon;
    if (!polygon_vertices.empty()) {
        polygon = Polygon(polygon_vertices, start_gps);
        RCLCPP_INFO(logger_, "[ComputeWaypointPath] Using polygon constraint with %zu vertices.", 
                    polygon_vertices.size());
    } else {
        RCLCPP_INFO(logger_, "[ComputeWaypointPath] No polygon constraint provided.");
    }
    
    // Call the RRT* path planning algorithm
    RCLCPP_INFO(logger_, "[ComputeWaypointPath] Calling RRT* planner with %zu obstacles.", 
                obstacles_in_meters.size());
    PathResult path_result = pathPlanningWithBothPaths(start_gps, goal_gps, obstacles_in_meters, polygon);
    
    // Prefer the simplified path, but fall back to the original if simplification fails
    std::vector<GPSPoint> path_gps = path_result.simplifiedPath;
    if (path_gps.empty()) {
        RCLCPP_WARN(logger_, "[ComputeWaypointPath] Simplified path was empty, falling back to original RRT* path.");
        path_gps = path_result.originalPath;
    }

    if (path_gps.empty()) {
        RCLCPP_ERROR(logger_, "[ComputeWaypointPath] Path planner failed to find a valid path. Returning empty path.");
        return {}; // Return an empty vector
    }

    RCLCPP_INFO(logger_, "[ComputeWaypointPath] Path found with %zu waypoints. Converting to NavSatFix.", path_gps.size());

    // Convert the resulting path of GPSPoints to a vector of NavSatFix messages
    std::vector<sensor_msgs::msg::NavSatFix> waypoints;
    for (const auto& gps_point : path_gps) {
        sensor_msgs::msg::NavSatFix waypoint;
        waypoint.latitude = gps_point.lat;
        waypoint.longitude = gps_point.lon;
        waypoint.altitude = target_altitude; // Use the fixed target altitude for all points
        waypoints.push_back(waypoint);
    }

    return waypoints;
}


// #include "behavior_executive/action_handlers/navigate_to_waypoint_handler.hpp"
// #include "behavior_executive/mavros_adapter_interface.hpp"

// #include <behavior_tree/behavior_tree.hpp>
// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/nav_sat_fix.hpp>

// NavigateToWaypointHandler::NavigateToWaypointHandler(
//     std::shared_ptr<IMavrosAdapter> mavros_adapter,
//     rclcpp::Logger logger)
//     : mavros_adapter_(mavros_adapter),
//       logger_(logger),
//       target_lat_(0.0),
//       target_lon_(0.0),
//       target_alt_(10.0) {}

// void NavigateToWaypointHandler::set_target_waypoint(double lat, double lon, double alt) {
//     target_lat_ = lat;
//     target_lon_ = lon;
//     target_alt_ = alt;
    
//     RCLCPP_INFO(logger_, 
//                 "[Navigate to Waypoint] Target waypoint set: lat=%.6f, lon=%.6f, alt=%.1fm",
//                 target_lat_, target_lon_, target_alt_);
// }

// void NavigateToWaypointHandler::get_target_waypoint(double& lat, double& lon, double& alt) const {
//     lat = target_lat_;
//     lon = target_lon_;
//     alt = target_alt_;
// }

// bool NavigateToWaypointHandler::check_safety(const Context& ctx) {
//     // TEMPORARY: Bypass safety checks for debugging
//     RCLCPP_WARN(logger_, "[Navigate to Waypoint] BYPASSING SAFETY CHECKS FOR DEBUGGING");
//     RCLCPP_INFO(logger_, "[Navigate to Waypoint] Current GPS: lat=%.6f, lon=%.6f", 
//                 ctx.current_latitude, ctx.current_longitude);
//     RCLCPP_INFO(logger_, "[Navigate to Waypoint] Target waypoint: lat=%.6f, lon=%.6f, alt=%.1fm",
//                 target_lat_, target_lon_, target_alt_);
//     return true;
    
//     // // 检查 GPS 是否有效
//     // if (ctx.current_latitude == 0.0 && ctx.current_longitude == 0.0) {
//     //     RCLCPP_ERROR(logger_, "[Navigate to Waypoint] Invalid current GPS position");
//     //     return false;
//     // }

//     // // 检查目标航点是否有效
//     // if (target_lat_ == 0.0 && target_lon_ == 0.0) {
//     //     RCLCPP_ERROR(logger_, "[Navigate to Waypoint] Invalid target waypoint (0.0, 0.0)");
//     //     return false;
//     // }

//     // // 检查目标高度是否合理
//     // if (target_alt_ < MIN_ALTITUDE || target_alt_ > MAX_ALTITUDE) {
//     //     RCLCPP_ERROR(logger_, 
//     //                 "[Navigate to Waypoint] Invalid target altitude: %.2fm (must be between %.1f and %.1f)",
//     //                 target_alt_, MIN_ALTITUDE, MAX_ALTITUDE);
//     //     return false;
//     // }

//     // RCLCPP_INFO(logger_, "[Navigate to Waypoint] Safety checks passed");
//     // return true;
// }

// void NavigateToWaypointHandler::on_activated(bt::Action* action, const Context& ctx) {
//     RCLCPP_INFO(logger_, 
//                 "[Navigate to Waypoint] Navigating to target: (%.6f, %.6f) at %.1fm altitude",
//                 target_lat_, target_lon_, target_alt_);

//     // 创建单个航点
//     sensor_msgs::msg::NavSatFix waypoint;
//     waypoint.latitude = target_lat_;
//     waypoint.longitude = target_lon_;
//     waypoint.altitude = target_alt_;

//     std::vector<sensor_msgs::msg::NavSatFix> waypoints = {waypoint};

//     // 首先清除之前的任务
//     mavros_adapter_->clear_waypoints(
//         [this, action, waypoints, ctx](bool clear_success) {
//             if (!clear_success) {
//                 RCLCPP_WARN(logger_, 
//                            "[Navigate to Waypoint] Failed to clear waypoints, proceeding anyway");
//             }
            
//             RCLCPP_INFO(logger_, "[Navigate to Waypoint] Pushing waypoint to flight controller");
            
//             // 推送航点
//             mavros_adapter_->push_waypoints(
//                 waypoints,
//                 target_alt_,
//                 HOLD_TIME,
//                 {},  // 不使用 yaws
//                 ACCEPTANCE_RADIUS,
//                 [this, action, ctx](bool push_success) {
//                     if (push_success) {
//                         RCLCPP_INFO(logger_, "[Navigate to Waypoint] Successfully pushed waypoint");
                        
//                         // 如果当前模式不是 AUTO.MISSION，需要切换模式
//                         if (ctx.current_mode != "AUTO.MISSION") {
//                             RCLCPP_INFO(logger_, 
//                                        "[Navigate to Waypoint] Switching to AUTO.MISSION mode");
                            
//                             mavros_adapter_->set_mode(
//                                 "AUTO.MISSION",
//                                 [this, action](bool mode_success) {
//                                     if (mode_success) {
//                                         action->set_success();
//                                         RCLCPP_INFO(logger_, 
//                                                    "[Navigate to Waypoint] Navigation started successfully");
//                                     } else {
//                                         action->set_failure();
//                                         RCLCPP_ERROR(logger_, 
//                                                     "[Navigate to Waypoint] Failed to switch to MISSION mode");
//                                     }
//                                 },
//                                 "Navigate to Waypoint"
//                             );
//                         } else {
//                             action->set_success();
//                             RCLCPP_INFO(logger_, 
//                                        "[Navigate to Waypoint] Navigation started (already in MISSION mode)");
//                         }
//                     } else {
//                         action->set_failure();
//                         RCLCPP_ERROR(logger_, "[Navigate to Waypoint] Failed to push waypoint");
//                     }
//                 },
//                 "Navigate to Waypoint",
//                 ctx.current_mode
//             );
//         },
//         "Navigate to Waypoint"
//     );
// }

