#include "behavior_executive/action_handlers/go_to_target_with_avoidance_handler.hpp"
#include "behavior_executive/mavros_adapter_interface.hpp"

#include <behavior_tree/behavior_tree.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

GoToTargetWithAvoidanceHandler::GoToTargetWithAvoidanceHandler(
    std::shared_ptr<IMavrosAdapter> mavros_adapter,
    // std::shared_ptr<rclcpp_action::Client<ComputePath>> path_planner_client,
    rclcpp::Logger logger)
    : mavros_adapter_(mavros_adapter),
      // path_planner_client_(path_planner_client),
      logger_(logger),
      target_position_({0.0, 0.0}) {}

void GoToTargetWithAvoidanceHandler::set_target_position(const std::array<double, 2>& casualty_position) {
    target_position_ = casualty_position;
}

bool GoToTargetWithAvoidanceHandler::check_safety(const Context& ctx) {
    if (ctx.current_latitude == 0.0 && ctx.current_longitude == 0.0) {
        RCLCPP_ERROR(logger_, "[GoToWithAvoidance] Invalid current GPS position.");
        return false;
    }
    if (target_position_[0] == 0.0 && target_position_[1] == 0.0) {
        RCLCPP_ERROR(logger_, "[GoToWithAvoidance] Invalid target position (0.0, 0.0).");
        return false;
    }
    RCLCPP_INFO(logger_, "[GoToWithAvoidance] Safety checks passed.");
    return true;
}

void GoToTargetWithAvoidanceHandler::on_activated(bt::Action* action, const Context& ctx) {
    RCLCPP_INFO(logger_, "[GoToWithAvoidance] Action activated. Requesting path to target at (%.6f, %.6f).",
                target_position_[0], target_position_[1]);

    // ====================================================================================
    // TODO: Replace this block with a call to the path_planner_node action server
    // ====================================================================================
    // This is a MOCK implementation that simulates the path planner.
    // It creates a simple two-point path: a midpoint and the final target.
    
    RCLCPP_WARN(logger_, "[GoToWithAvoidance] MOCKING path planner. Using a simple 2-point path.");

    // Create a dummy path
    std::vector<sensor_msgs::msg::NavSatFix> path;
    sensor_msgs::msg::NavSatFix current_pos;
    current_pos.latitude = ctx.current_latitude;
    current_pos.longitude = ctx.current_longitude;

    // 1. Midpoint (dummy obstacle avoidance)
    sensor_msgs::msg::NavSatFix midpoint;
    midpoint.latitude = ctx.current_latitude + (target_position_[0] - ctx.current_latitude) * 0.5 + 0.0001; // Offset to simulate avoidance
    midpoint.longitude = ctx.current_longitude + (target_position_[1] - ctx.current_longitude) * 0.5;
    path.push_back(midpoint);

    // 2. Final target
    sensor_msgs::msg::NavSatFix final_target;
    final_target.latitude = target_position_[0];
    final_target.longitude = target_position_[1];
    path.push_back(final_target);

    // Use the path to push waypoints
    RCLCPP_INFO(logger_, "[GoToWithAvoidance] Pushing %zu waypoints to flight controller at fixed altitude.", path.size());
    mavros_adapter_->clear_waypoints(
        [this, action, path, ctx](bool clear_success) {
            if (!clear_success) {
                RCLCPP_WARN(logger_, "[GoToWithAvoidance] Failed to clear waypoints, proceeding anyway.");
            }
            
            this->mavros_adapter_->push_waypoints(
                path,
                ctx.target_altitude, // Use current altitude as fixed flight altitude
                this->HOLD_TIME,
                {}, // No yaws
                this->ACCEPTANCE_RADIUS,
                [this, action, ctx](bool push_success) {
                    if (push_success) {
                        if (ctx.current_mode != "AUTO.MISSION") {
                            this->mavros_adapter_->set_mode("AUTO.MISSION", [action](bool mode_success) {
                                if (mode_success) action->set_success();
                                else action->set_failure();
                            }, action->get_label());
                        } else {
                            action->set_success();
                        }
                    } else {
                        action->set_failure();
                    }
                },
                action->get_label(),
                ctx.current_mode
            );
        },
        action->get_label()
    );
    // ====================================================================================
    // End of MOCK block
    // ====================================================================================
}
