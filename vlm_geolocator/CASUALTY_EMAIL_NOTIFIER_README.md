# Casualty Email Notifier

自动邮件通知系统，当检测到伤员位置时立即发送邮件警报。

## 功能特性

✅ **实时监听** `/casualty_geolocated` 话题  
✅ **自动发送邮件** 包含 GPS 坐标和 Google Maps 链接  
✅ **精美 HTML 格式** 清晰的警报界面  
✅ **高优先级标记** 邮件客户端会突出显示  
✅ **线程安全** 不会阻塞 ROS2 节点  
✅ **多收件人支持** 可同时通知多个人员  

## 快速开始

### 1. 配置邮件设置

编辑 `config/email_config.yaml`：

```yaml
smtp:
  server: "smtp.gmail.com"
  port: 587
  use_tls: true
  username: "your-email@gmail.com"
  password: "your-app-password"  # Gmail 应用专用密码

sender:
  email: "your-email@gmail.com"
  name: "Chiron Casualty Detection System"

recipients:
  - "medic1@example.com"
  - "command@example.com"
```

### 2. Gmail 应用密码设置（推荐）

如果使用 Gmail，需要创建应用专用密码：

1. 访问 [Google Account Security](https://myaccount.google.com/security)
2. 启用 **两步验证**
3. 搜索 "应用密码" (App Passwords)
4. 选择 "邮件" 和 "其他设备"
5. 复制生成的 16 位密码到 `email_config.yaml`

### 3. 运行节点

```bash
cd /home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator

# 设置 ROS2 domain
export ROS_DOMAIN_ID=100

# Source ROS2
source /opt/ros/humble/setup.bash

# 运行邮件通知节点
python3 src/casualty_email_notifier.py
```

### 4. 测试发送

在另一个终端测试：

```bash
export ROS_DOMAIN_ID=100
source /opt/ros/humble/setup.bash

# 发布测试 GPS 位置
ros2 topic pub --once /casualty_geolocated sensor_msgs/msg/NavSatFix "{
  latitude: 33.775600,
  longitude: -84.396300,
  altitude: 100.0,
  status: {status: 0}
}"
```

你应该会收到一封邮件！

## 邮件内容示例

邮件包含以下信息：

### 📧 邮件主题
```
🚨 CASUALTY DETECTED - 2025-11-16 14:30:45
```

### 📋 邮件内容
- **检测时间**: 2025-11-16 14:30:45
- **伤员编号**: #1
- **GPS 坐标**:
  - 纬度: 33.775600°
  - 经度: -84.396300°
  - 海拔: 100.0 m
- **Google Maps 链接**: 一键打开地图导航
- **行动提示**: 请立即派遣医疗援助

### 🎨 格式
- **HTML 格式**: 精美的红色警报设计
- **纯文本备份**: 兼容所有邮件客户端
- **高优先级**: 邮件会被标记为重要

## 集成到 FVD.sh

将邮件通知节点添加到自动启动脚本：

```bash
# Edit ~/FVD.sh
# Add after Pane 6 (Video Merger):

# Split to create 8th pane for Email Notifier
tmux select-pane -t 5
tmux split-window -h

# Pane 7: Casualty Email Notifier
tmux select-pane -t 7
sleep 5
tmux send-keys "cd /home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator" Enter
sleep 1
tmux send-keys "export ROS_DOMAIN_ID=100 && source /opt/ros/humble/setup.bash && python3 src/casualty_email_notifier.py" Enter

# Update pane title
tmux select-pane -t 7 -T "Email Notifier (D100)"
```

同时在清理部分添加：

```bash
# Kill email notifier
echo "  - Stopping casualty_email_notifier..."
pkill -f "casualty_email_notifier.py" 2>/dev/null
```

## 其他 SMTP 服务器

### Outlook/Hotmail
```yaml
smtp:
  server: "smtp-mail.outlook.com"
  port: 587
  use_tls: true
```

### Yahoo Mail
```yaml
smtp:
  server: "smtp.mail.yahoo.com"
  port: 587
  use_tls: true
```

### 自定义 SMTP 服务器
```yaml
smtp:
  server: "mail.yourdomain.com"
  port: 465
  use_tls: false  # Use SSL
```

## 故障排查

### 问题 1: "Authentication failed"
**原因**: 密码错误或需要应用专用密码  
**解决**: 使用 Gmail 应用密码而不是账户密码

### 问题 2: "Connection refused"
**原因**: SMTP 端口或服务器错误  
**解决**: 检查 `smtp.server` 和 `smtp.port` 配置

### 问题 3: "No emails received"
**原因**: 收件人地址错误或邮件被过滤到垃圾箱  
**解决**: 
- 检查 `recipients` 列表
- 查看垃圾邮件文件夹
- 将发件人添加到安全发件人列表

### 问题 4: 节点日志查看
```bash
# 查看节点输出
ros2 run ... casualty_email_notifier.py

# 或在 tmux 中查看对应窗格
tmux attach -t chiron_ops_panes
# 使用 Ctrl+b 然后箭头键切换到 Email Notifier 窗格
```

## 高级功能

### 速率限制（防止邮件轰炸）

在 `email_config.yaml` 中添加：

```yaml
rate_limiting:
  enabled: true
  min_interval_seconds: 10  # 最少 10 秒发送一次
```

然后修改代码实现速率限制逻辑。

### 添加图片附件

可以附加伤员检测的视频帧截图（需要修改代码）。

### SMS 通知

未来可以集成 Twilio 等服务发送短信。

## 安全提示

⚠️ **不要将 email_config.yaml 提交到 Git！**

```bash
# 添加到 .gitignore
echo "vlm_geolocator/config/email_config.yaml" >> .gitignore
```

🔒 **使用应用专用密码**，不要使用账户主密码

🛡️ **限制收件人列表**，只发送给授权人员

## 依赖项

所有依赖项已包含在标准 Python 库中：
- `smtplib` - SMTP 邮件发送
- `email` - 邮件格式化
- `yaml` - 配置文件解析
- `rclpy` - ROS2 Python 客户端

无需安装额外包！

## 架构

```
┌─────────────────────────────────────┐
│  VLM Geolocator Node (Domain 100)   │
│  (vision_inference_node_refactored) │
└────────────┬────────────────────────┘
             │ publishes
             ▼
    /casualty_geolocated (NavSatFix)
             │
             │ subscribes
             ▼
┌─────────────────────────────────────┐
│  Casualty Email Notifier Node       │
│  (casualty_email_notifier.py)       │
└────────────┬────────────────────────┘
             │ sends
             ▼
     📧 Email via SMTP
             │
             ▼
┌─────────────────────────────────────┐
│  Recipients                         │
│  - Medics                           │
│  - Command Center                   │
│  - Emergency Response               │
└─────────────────────────────────────┘
```

## 日志示例

```
[INFO] [casualty_email_notifier]: 📧 Casualty Email Notifier started
[INFO] [casualty_email_notifier]:    SMTP Server: smtp.gmail.com:587
[INFO] [casualty_email_notifier]:    Recipients: medic1@example.com, command@example.com
[INFO] [casualty_email_notifier]:    From: chiron@example.com
[INFO] [casualty_email_notifier]:    Listening to: /casualty_geolocated
[INFO] [casualty_email_notifier]: 📍 Casualty #1 detected: lat=33.775600, lon=-84.396300, alt=100.0m
[INFO] [casualty_email_notifier]: ✅ Email notification #1 sent successfully
```

## 许可

与 Chiron 项目相同的许可证。



