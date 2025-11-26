#!/usr/bin/env bash
# 飞行日志查看工具
# 用于查看和分析飞行后的日志文件

set -euo pipefail

CONTAINER_NAME="airstack-unified"
LOG_DIR_CONTAINER="/root/.ros/log"
LOG_DIR_HOST="${HOME}/.ros/flight_logs"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== 飞行日志查看工具 ===${NC}"
echo ""

# 检查容器是否运行
if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
    echo -e "${YELLOW}警告: 容器 ${CONTAINER_NAME} 未运行${NC}"
    echo "将尝试从之前停止的容器中提取日志..."
fi

# 创建主机日志目录
mkdir -p "${LOG_DIR_HOST}"

# 函数：从容器复制日志到主机
copy_logs_from_container() {
    echo -e "${GREEN}正在从容器复制日志文件...${NC}"
    
    # 获取容器 ID（无论运行或停止）
    CONTAINER_ID=$(docker ps -a --filter "name=${CONTAINER_NAME}" --format '{{.ID}}' | head -n1)
    
    if [ -z "${CONTAINER_ID}" ]; then
        echo -e "${RED}错误: 找不到容器 ${CONTAINER_NAME}${NC}"
        exit 1
    fi
    
    # 复制日志文件
    if docker cp "${CONTAINER_ID}:${LOG_DIR_CONTAINER}" "${LOG_DIR_HOST}/latest" 2>/dev/null; then
        echo -e "${GREEN}✓ 日志文件已复制到: ${LOG_DIR_HOST}/latest${NC}"
        return 0
    else
        echo -e "${YELLOW}警告: 无法复制日志文件（可能还没有日志）${NC}"
        return 1
    fi
}

# 函数：列出所有日志文件
list_logs() {
    echo -e "${BLUE}可用的日志文件:${NC}"
    echo ""
    
    # ROS 2 日志
    if [ -d "${LOG_DIR_HOST}/latest" ]; then
        echo -e "${GREEN}ROS 2 系统日志:${NC}"
        find "${LOG_DIR_HOST}/latest" -type f -name "*.log" | sort -r | head -10 | while read -r log_file; do
            size=$(du -h "$log_file" | cut -f1)
            mod_time=$(stat -c %y "$log_file" | cut -d'.' -f1)
            echo "  📄 $(basename $log_file) [${size}, ${mod_time}]"
        done
        echo ""
    fi
    
    # BehaviorLogger CSV 文件
    if [ -d "${HOME}/.ros/transitions" ]; then
        echo -e "${GREEN}行为树转换日志 (BehaviorLogger):${NC}"
        find "${HOME}/.ros/transitions" -type f -name "*.csv" | sort -r | head -5 | while read -r csv_file; do
            size=$(du -h "$csv_file" | cut -f1)
            mod_time=$(stat -c %y "$csv_file" | cut -d'.' -f1)
            lines=$(wc -l < "$csv_file")
            echo "  📊 $(basename $csv_file) [${size}, ${lines} 行, ${mod_time}]"
        done
        echo ""
    fi
}

# 函数：查看特定节点的日志
view_node_log() {
    local node_name=$1
    local log_files
    
    if [ -d "${LOG_DIR_HOST}/latest" ]; then
        log_files=$(find "${LOG_DIR_HOST}/latest" -type f -name "*${node_name}*.log" | head -1)
        
        if [ -n "$log_files" ]; then
            echo -e "${GREEN}查看节点 '${node_name}' 的日志:${NC}"
            echo "文件: ${log_files}"
            echo ""
            less +G "$log_files"  # +G 跳到文件末尾
        else
            echo -e "${RED}错误: 找不到节点 '${node_name}' 的日志文件${NC}"
        fi
    else
        echo -e "${RED}错误: 日志目录不存在，请先运行 'copy' 命令${NC}"
    fi
}

# 函数：搜索日志中的关键字
search_logs() {
    local keyword=$1
    
    echo -e "${GREEN}搜索包含 '${keyword}' 的日志:${NC}"
    echo ""
    
    if [ -d "${LOG_DIR_HOST}/latest" ]; then
        grep -r --color=always "$keyword" "${LOG_DIR_HOST}/latest" | head -50
    else
        echo -e "${RED}错误: 日志目录不存在，请先运行 'copy' 命令${NC}"
    fi
}

# 函数：查看行为树转换日志
view_behavior_log() {
    local latest_csv
    
    if [ -d "${HOME}/.ros/transitions" ]; then
        latest_csv=$(find "${HOME}/.ros/transitions" -type f -name "*.csv" | sort -r | head -1)
        
        if [ -n "$latest_csv" ]; then
            echo -e "${GREEN}查看最新的行为树转换日志:${NC}"
            echo "文件: ${latest_csv}"
            echo ""
            
            # 使用 column 格式化 CSV 输出
            if command -v column &> /dev/null; then
                less -S <(cat "$latest_csv" | column -t -s',')
            else
                less -S "$latest_csv"
            fi
        else
            echo -e "${RED}错误: 找不到行为树日志文件${NC}"
        fi
    else
        echo -e "${YELLOW}警告: 行为树日志目录不存在: ${HOME}/.ros/transitions${NC}"
    fi
}

# 主菜单
show_menu() {
    echo ""
    echo -e "${BLUE}可用命令:${NC}"
    echo "  copy              - 从容器复制日志到主机"
    echo "  list              - 列出所有可用日志文件"
    echo "  view <node>       - 查看特定节点的日志 (例如: view behavior_executive)"
    echo "  search <keyword>  - 在日志中搜索关键字"
    echo "  behavior          - 查看行为树转换日志 (BehaviorLogger CSV)"
    echo "  help              - 显示此帮助信息"
    echo ""
}

# 解析命令行参数
case "${1:-help}" in
    copy)
        copy_logs_from_container
        ;;
    list)
        copy_logs_from_container
        list_logs
        ;;
    view)
        if [ $# -lt 2 ]; then
            echo -e "${RED}错误: 请指定节点名称${NC}"
            echo "用法: $0 view <node_name>"
            exit 1
        fi
        view_node_log "$2"
        ;;
    search)
        if [ $# -lt 2 ]; then
            echo -e "${RED}错误: 请指定搜索关键字${NC}"
            echo "用法: $0 search <keyword>"
            exit 1
        fi
        copy_logs_from_container
        search_logs "$2"
        ;;
    behavior)
        view_behavior_log
        ;;
    help|*)
        show_menu
        ;;
esac

