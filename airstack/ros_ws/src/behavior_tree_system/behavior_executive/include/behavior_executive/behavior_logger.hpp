#pragma once

#include <fstream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

/**
 * @brief 行为状态记录器
 *
 * 负责记录行为树中动作与条件的状态变化：
 * - Action 的激活、运行、成功、失败
 * - Condition 的布尔变化
 * - 相关条件的快照、失败原因和附加信息
 *
 * 输出位置：
 * 1. ROS 日志 (RCLCPP_INFO)
 * 2. CSV 文件（便于离线分析）
 */
class BehaviorLogger {
public:
    /**
     * @brief 构造函数
     * @param node ROS 节点指针，用于日志输出
     * @param log_dir 日志文件目录（默认：~/.ros/transitions/）
     * @param log_to_console 是否输出到控制台
     * @param log_to_file 是否输出到文件
     */
    explicit BehaviorLogger(
        rclcpp::Node* node,
        const std::string& log_dir = "",
        bool log_to_console = true,
        bool log_to_file = true);

    ~BehaviorLogger();

    /**
     * @brief 记录 Action 状态变化
     * @param action_name Action 名称
     * @param status 状态（ACTIVATED, RUNNING, SUCCESS, FAILURE）
     * @param conditions 相关 conditions 状态快照
     * @param failure_reason 失败原因（如适用）
     * @param additional_info 额外信息
     */
    void log_action_transition(
        const std::string& action_name,
        const std::string& status,
        const std::map<std::string, bool>& conditions,
        const std::string& failure_reason = "",
        const std::string& additional_info = "");

    /**
     * @brief 记录 Condition 变化
     * @param condition_name Condition 名称
     * @param old_value 旧值
     * @param new_value 新值
     */
    void log_condition_change(
        const std::string& condition_name,
        bool old_value,
        bool new_value);

    /**
     * @brief 刷新缓冲区，确保数据写入文件
     */
    void flush();

    /**
     * @brief 获取日志文件路径
     */
    std::string get_log_file_path() const { return log_file_path_; }

private:
    /**
     * @brief 获取当前时间戳字符串
     * @return 格式化的时间戳（YYYY-MM-DD HH:MM:SS.mmm）
     */
    std::string get_timestamp() const;

    /**
     * @brief 将 map 转换为 JSON 风格字符串
     */
    std::string conditions_to_string(const std::map<std::string, bool>& conditions) const;

    /**
     * @brief 转义 CSV 字段中的特殊字符
     */
    std::string escape_csv_field(const std::string& field) const;

    /**
     * @brief 写入 CSV 头部
     */
    void write_csv_header();

    rclcpp::Node* node_;
    std::string log_file_path_;
    std::ofstream log_file_;
    std::mutex file_mutex_;

    bool log_to_console_;
    bool log_to_file_;
    bool file_initialized_;
};


