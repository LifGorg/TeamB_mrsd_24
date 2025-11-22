#pragma once

#include "action_handler_interface.hpp"
#include <memory>
#include <vector>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include "behavior_executive/mission_planning/path_planning.hpp"

// Forward declarations
class IMavrosAdapter;

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Navigate to Waypoint 动作处理器
 * 
 * 封装导航到指定航点的任务逻辑：
 * 1. 接收 GCS 选择的目标航点
 * 2. 验证航点有效性
 * 3. 通过 MavrosAdapter 推送航点到飞控
 * 4. 切换到 AUTO.MISSION 模式
 */
class NavigateToWaypointHandler : public IActionHandler {
public:
    /**
     * @brief 构造函数（依赖注入）
     * @param mavros_adapter MAVROS 通信适配器
     * @param logger ROS logger
     */
    NavigateToWaypointHandler(
        std::shared_ptr<IMavrosAdapter> mavros_adapter,
        rclcpp::Logger logger);

    /**
     * @brief 设置目标航点
     * @param lat 目标纬度
     * @param lon 目标经度
     * @param alt 目标高度（米）
     */
    void set_target_waypoint(double lat, double lon, double alt);

    /**
     * @brief 获取当前目标航点
     * @param lat 输出：纬度
     * @param lon 输出：经度
     * @param alt 输出：高度
     */
    void get_target_waypoint(double& lat, double& lon, double& alt) const;

    // 实现 IActionHandler 接口
    void on_activated(bt::Action* action, const Context& ctx) override;
    bool check_safety(const Context& ctx) override;

    /**
     * @brief Computes a safe path considering obstacles.
     * 
     * This function implements the RRT* algorithm to compute a multi-point path
     * from a start to a goal location, avoiding a given list of obstacles.
     * 
     * @param start_gps The current position of the drone.
     * @param goal_gps The final target position (e.g., casualty location).
     * @param obstacles A vector of obstacles, each with a GPS location and a radius in meters.
     * @param target_altitude The desired constant altitude for the entire path.
     * @return A vector of NavSatFix messages representing the safe path. Returns an empty vector if no path is found.
     */
    std::vector<sensor_msgs::msg::NavSatFix> compute_waypoint_path(
        const GPSPoint& start_gps,
        const GPSPoint& goal_gps,
        const std::vector<Obstacle>& obstacles,
        double target_altitude
    );

private:
    std::shared_ptr<IMavrosAdapter> mavros_adapter_;
    rclcpp::Logger logger_;

    // 目标航点坐标
    double target_lat_;
    double target_lon_;
    double target_alt_;

    // 任务参数
    static constexpr double HOLD_TIME = 5.0;              // 秒：在航点停留时间
    static constexpr double ACCEPTANCE_RADIUS = 2.0;      // 米：航点接受半径
    static constexpr double MIN_ALTITUDE = 5.0;           // 米：最低安全高度
    static constexpr double MAX_ALTITUDE = 120.0;         // 米：最高飞行高度
};

