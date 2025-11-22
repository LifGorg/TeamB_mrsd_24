#pragma once

#include "behavior_executive/action_handlers/action_handler_interface.hpp"
#include <memory>

// Forward declarations
class IMavrosAdapter;
namespace rclcpp_action {
    template<typename ActionT>
    class Client;
}
// #include "custom_interfaces/action/compute_path.hpp" // Placeholder for the Action definition

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Go To Target With Avoidance 动作处理器
 * 
 * 封装带避障功能的“就位”任务逻辑：
 * 1. 接收最新的伤员位置作为目标
 * 2. 以当前高度为固定飞行高度
 * 3. 调用路径规划服务，获取一条能避开所有已知障碍物的安全路径
 * 4. 将完整路径推送给飞控执行
 */
class GoToTargetWithAvoidanceHandler : public IActionHandler {
public:
    // using ComputePath = custom_interfaces::action::ComputePath; // Action Type
    // using GoalHandleComputePath = rclcpp_action::ClientGoalHandle<ComputePath>;

    GoToTargetWithAvoidanceHandler(
        std::shared_ptr<IMavrosAdapter> mavros_adapter,
        // std::shared_ptr<rclcpp_action::Client<ComputePath>> path_planner_client,
        rclcpp::Logger logger);

    // IActionHandler interface implementation
    void on_activated(bt::Action* action, const Context& ctx) override;
    bool check_safety(const Context& ctx) override;

    void set_target_position(const std::array<double, 2>& casualty_position);

private:
    std::shared_ptr<IMavrosAdapter> mavros_adapter_;
    // std::shared_ptr<rclcpp_action::Client<ComputePath>> path_planner_client_;
    rclcpp::Logger logger_;

    std::array<double, 2> target_position_;

    // Constants
    static constexpr double HOLD_TIME = 2.0;
    static constexpr double ACCEPTANCE_RADIUS = 0.5;
};
