#!/bin/bash

# 配置远程机器网络脚本
# 用途：设置远程机器使用本地机器作为网关

echo "正在配置远程机器网络设置..."

# 1. 删除现有的默认路由
echo "1. 删除现有默认路由"
sshpass -p 'passme24' ssh dtc@10.3.1.32 'sudo ip route del default'

# 2. 添加新的默认路由，指向本地机器
echo "2. 添加新的默认路由 (网关: 10.3.1.5)"
sshpass -p 'passme24' ssh dtc@10.3.1.32 'sudo ip route add default via 10.3.1.5'

# 3. 设置DNS服务器
echo "3. 设置DNS服务器"
sshpass -p 'passme24' ssh dtc@10.3.1.32 'echo "nameserver 8.8.8.8" | sudo tee /etc/resolv.conf > /dev/null'
sshpass -p 'passme24' ssh dtc@10.3.1.32 'echo "nameserver 8.8.4.4" | sudo tee -a /etc/resolv.conf > /dev/null'

echo "远程机器网络配置完成！"
echo ""
echo "现在可以测试远程机器的互联网连接："
echo "sshpass -p 'passme24' ssh dtc@10.3.1.32 'ping -c 3 8.8.8.8'"
