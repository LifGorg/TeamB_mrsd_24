# Gemini 视频分析器 - 伤员检测系统

## 概述

这是一个基于 Google Gemini API 的视频理解管道，专门用于分析无人机视频中的伤员情况。系统能够检测：

- ✅ 伤员识别（是否为伤员）
- ✅ 创伤评估（头部、躯干、四肢）
- ✅ 出血严重程度（无/轻微/中度/严重/危急）
- ✅ 关键时刻标记（时间戳）
- ✅ 详细医疗建议

## 特性

- **智能提示注入**: 使用专门设计的医疗评估提示词，引导模型进行专业的伤员评估
- **抖动视频处理**: 针对无人机视频优化，能够处理有抖动的航拍画面
- **结构化输出**: 返回标准化的JSON格式结果，便于集成到其他系统
- **时间戳标记**: 自动标记关键发现的视频时间点
- **可定制分析**: 支持自定义FPS、时间片段、提示词等

## 安装

### 1. 安装依赖

```bash
cd /home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator
pip install -r requirements.txt
```

主要依赖：
- `google-genai>=1.0.0` - Google Gemini API 客户端
- 其他现有依赖

### 2. 获取 Gemini API 密钥

1. 访问 [Google AI Studio](https://aistudio.google.com/app/apikey)
2. 创建或选择一个项目
3. 生成 API 密钥

### 3. 设置环境变量

```bash
export GEMINI_API_KEY='your-api-key-here'
```

或者在代码中直接传递 API 密钥。

## 快速开始

### 方法 1: 使用测试脚本

```bash
cd /home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator

# 确保设置了 API 密钥
export GEMINI_API_KEY='your-api-key-here'

# 运行测试脚本
python test_gemini_analyzer.py
```

脚本会自动分析示例视频：
`/home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator/inference_logs/recordings/recording_20251113_164742.mp4`

### 方法 2: 在代码中使用

```python
from vlm_geolocator.vision.gemini_video_analyzer import (
    GeminiVideoAnalyzer,
    TraumaLocation,
    HemorrhageSeverity
)

# 创建分析器
analyzer = GeminiVideoAnalyzer(api_key="your-api-key")

# 分析视频
analysis = analyzer.analyze_video(
    video_path="/path/to/drone_video.mp4",
    fps=1.0  # 每秒采样1帧
)

# 打印结果
analyzer.print_analysis(analysis)

# 检查结果
if analysis.is_casualty:
    print(f"检测到伤员！置信度: {analysis.confidence:.0%}")
    print(f"创伤位置: {[loc.value for loc in analysis.trauma_locations]}")
    print(f"出血程度: {analysis.hemorrhage_severity.value}")
```

### 方法 3: 命令行使用

```bash
cd /home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator/src/vlm_geolocator/vision

# 使用环境变量中的API密钥
python gemini_video_analyzer.py /path/to/video.mp4

# 或直接传递API密钥
python gemini_video_analyzer.py /path/to/video.mp4 your-api-key
```

## 高级用法

### 1. 自定义帧率采样

对于长视频，可以降低采样率以节省处理时间：

```python
# 每2秒采样1帧 (0.5 FPS)
analysis = analyzer.analyze_video(
    video_path="long_video.mp4",
    fps=0.5
)

# 每秒采样2帧 (2 FPS) - 用于快速移动场景
analysis = analyzer.analyze_video(
    video_path="fast_motion.mp4",
    fps=2.0
)
```

### 2. 分析特定时间段

只分析视频的特定片段：

```python
# 分析40秒到80秒之间的片段
analysis = analyzer.analyze_video(
    video_path="video.mp4",
    start_offset="40s",
    end_offset="80s"
)

# 使用分:秒格式
analysis = analyzer.analyze_video(
    video_path="video.mp4",
    start_offset="1:20",  # 1分20秒
    end_offset="2:15"     # 2分15秒
)
```

### 3. 自定义提示词

如果需要特定类型的分析，可以自定义提示词：

```python
custom_prompt = """
请分析这个视频，重点关注：
1. 是否有人员倒地
2. 是否有明显的出血
3. 人员的意识状态
请以简洁的方式回答。
"""

analysis = analyzer.analyze_video(
    video_path="video.mp4",
    custom_prompt=custom_prompt
)
```

### 4. 批量处理视频

```python
import os
import glob

analyzer = GeminiVideoAnalyzer()

# 获取所有视频文件
video_dir = "inference_logs/recordings/"
video_files = glob.glob(os.path.join(video_dir, "*.mp4"))

results = []
for video_path in video_files:
    print(f"分析: {video_path}")
    try:
        analysis = analyzer.analyze_video(video_path, fps=1.0)
        results.append({
            'video': video_path,
            'is_casualty': analysis.is_casualty,
            'confidence': analysis.confidence
        })
        analyzer.print_analysis(analysis)
    except Exception as e:
        print(f"  错误: {e}")

# 保存批量结果
import json
with open('batch_analysis_results.json', 'w') as f:
    json.dump(results, f, indent=2)
```

## 输出格式

### CasualtyAnalysis 对象

分析结果包含以下字段：

```python
@dataclass
class CasualtyAnalysis:
    is_casualty: bool              # 是否为伤员
    confidence: float              # 置信度 (0.0-1.0)
    trauma_locations: List[TraumaLocation]  # 创伤位置列表
    hemorrhage_severity: HemorrhageSeverity # 出血严重程度
    detailed_findings: Dict[str, str]       # 详细发现
    timestamps: List[str]          # 关键时刻列表 (MM:SS格式)
    raw_response: str              # Gemini原始响应
```

### 枚举类型

```python
class TraumaLocation(Enum):
    HEAD = "头部"
    TRUNK = "躯干"
    LIMBS = "四肢"
    MULTIPLE = "多处"
    NONE = "无"

class HemorrhageSeverity(Enum):
    NONE = "无出血"
    MINOR = "轻微出血"
    MODERATE = "中度出血"
    SEVERE = "严重出血"
    CRITICAL = "危急出血"
```

### JSON 输出示例

```json
{
  "video_path": "/path/to/video.mp4",
  "is_casualty": true,
  "confidence": 0.85,
  "trauma_locations": ["头部", "四肢"],
  "hemorrhage_severity": "中度出血",
  "detailed_findings": {
    "trauma": {
      "head": "头部可见明显伤口",
      "trunk": "无",
      "limbs": "右臂有擦伤"
    },
    "hemorrhage": {
      "severity": "中度出血",
      "location": "头部和右臂"
    },
    "position": "躺卧",
    "movement": false,
    "observations": "人员处于静止状态，需要紧急医疗援助",
    "overall": "检测到一名伤员，需要立即医疗响应",
    "recommendations": "派遣医疗团队，准备头部创伤处理设备和止血物资"
  },
  "timestamps": ["00:15", "00:42", "01:08"]
}
```

## 提示词工程

系统使用专门设计的医疗评估提示词，包含以下关键指导：

1. **角色设定**: 将模型设定为"经验丰富的紧急医疗响应专家"
2. **任务明确**: 清晰说明需要评估的四个维度（伤员识别、创伤、出血、时间戳）
3. **结构化输出**: 要求以JSON格式返回结果
4. **细节指导**: 提供每个评估项目的具体标准
5. **处理特殊情况**: 说明如何处理抖动、模糊等视频质量问题

您可以查看源代码中的 `MEDICAL_ASSESSMENT_PROMPT` 变量以了解完整提示词。

## 性能优化

### 1. 降低FPS处理长视频

```python
# 对于>5分钟的视频，使用0.5 FPS
if video_duration > 300:  # 秒
    fps = 0.5
else:
    fps = 1.0
```

### 2. 使用时间片段分析

```python
# 将长视频分段处理
duration = 300  # 假设5分钟视频
segment_length = 60  # 每段60秒

for i in range(0, duration, segment_length):
    start = f"{i}s"
    end = f"{min(i + segment_length, duration)}s"
    
    analysis = analyzer.analyze_video(
        video_path="long_video.mp4",
        start_offset=start,
        end_offset=end
    )
    # 处理分析结果...
```

### 3. 缓存上传的文件

分析器会自动缓存已上传的文件，重复分析同一视频时会自动使用缓存：

```python
# 第一次分析 - 会上传文件
analysis1 = analyzer.analyze_video("video.mp4", fps=1.0)

# 第二次分析 - 使用缓存，不会重新上传
analysis2 = analyzer.analyze_video("video.mp4", fps=2.0)

# 强制重新上传
analyzer.uploaded_files.clear()  # 清除缓存
```

## 成本考虑

Gemini API 的定价基于处理的token数量：

- **视频处理**: 约300 tokens/秒 (默认分辨率) 或 100 tokens/秒 (低分辨率)
- **音频处理**: 32 tokens/秒
- **文本输出**: 按生成的文本计费

### 成本优化建议：

1. **降低FPS**: 从1.0降到0.5可以减少约50%的视频token
2. **使用时间片段**: 只分析关键片段
3. **批量处理**: 一次API调用处理多个查询
4. **使用合适的模型**: `gemini-2.5-flash` 比 `gemini-2.5-pro` 更便宜

## 集成到现有系统

### 与 ROS2 集成

```python
import rclpy
from rclpy.node import Node
from vlm_geolocator.vision.gemini_video_analyzer import GeminiVideoAnalyzer

class CasualtyDetectionNode(Node):
    def __init__(self):
        super().__init__('casualty_detection_node')
        self.analyzer = GeminiVideoAnalyzer()
        
    def analyze_recorded_video(self, video_path):
        analysis = self.analyzer.analyze_video(video_path)
        
        if analysis.is_casualty:
            self.get_logger().warn(
                f"检测到伤员! 置信度: {analysis.confidence:.0%}"
            )
            # 发布消息到其他节点...
        
        return analysis
```

### 与现有的 Isaac 检测器结合

```python
# 在 vision_inference_node_refactored.py 中添加
from vlm_geolocator.vision.gemini_video_analyzer import GeminiVideoAnalyzer

class VisionInferenceNode:
    def __init__(self):
        # 现有初始化...
        self.gemini_analyzer = GeminiVideoAnalyzer()
    
    def analyze_recorded_video_with_gemini(self, video_path):
        """使用Gemini进行二次确认"""
        analysis = self.gemini_analyzer.analyze_video(video_path)
        return analysis
```

## 故障排除

### 问题 1: API密钥错误

```
错误: 未找到API密钥
```

**解决方案**:
```bash
export GEMINI_API_KEY='your-api-key-here'
```

### 问题 2: 视频上传失败

```
错误: 视频上传失败
```

**可能原因**:
- 文件过大 (>2GB)
- 网络连接问题
- 不支持的视频格式

**解决方案**:
- 压缩视频
- 检查网络连接
- 转换为支持的格式 (MP4, MOV, AVI等)

### 问题 3: 分析超时

```
错误: Request timeout
```

**解决方案**:
- 降低FPS
- 分析较短的时间片段
- 检查视频时长是否过长

### 问题 4: 无法解析响应

```
警告: 无法解析为JSON格式
```

**可能原因**:
- 模型返回了非结构化文本
- 提示词需要优化

**解决方案**:
- 查看 `raw_response` 字段
- 调整提示词以强调JSON格式输出

## 技术细节

### 支持的视频格式

- MP4 (`video/mp4`)
- MPEG (`video/mpeg`)
- MOV (`video/mov`)
- AVI (`video/avi`)
- FLV (`video/x-flv`)
- MPG (`video/mpg`)
- WebM (`video/webm`)
- WMV (`video/wmv`)
- 3GP (`video/3gpp`)

### 模型信息

- **默认模型**: `gemini-2.5-flash`
- **上下文窗口**: 1M tokens (可处理约1小时视频)
- **默认采样率**: 1 FPS
- **默认音频**: 1 Kbps

## 示例输出

```
============================================================
伤员分析结果
============================================================

是否为伤员: 是
置信度: 85.00%

创伤位置:
  - 头部
  - 四肢

出血严重程度: 中度出血

关键时间点:
  - 00:15
  - 00:42
  - 01:08

详细发现:
  trauma:
    head: 头部可见明显伤口
    trunk: 无
    limbs: 右臂有擦伤
  hemorrhage:
    severity: 中度出血
    location: 头部和右臂
  position: 躺卧
  movement: False
  observations: 人员处于静止状态，需要紧急医疗援助
  overall: 检测到一名伤员，需要立即医疗响应
  recommendations: 派遣医疗团队，准备头部创伤处理设备和止血物资
============================================================

============================================================
紧急响应建议
============================================================

⚠️  检测到伤员！
   置信度: 85%

🔴 头部创伤 - 优先级：极高
   建议: 立即派遣医疗团队，准备头部创伤处理设备

🟡 四肢创伤 - 优先级：中
   建议: 准备夹板和止血带

🟡 中度出血
   建议: 派遣医疗团队进行止血处理

关键时刻:
   ⏱️  00:15
   ⏱️  00:42
   ⏱️  01:08

============================================================

✅ 分析结果已保存到: recording_20251113_164742_gemini_analysis.json
```

## 参考资料

- [Gemini API 视频理解文档](https://ai.google.dev/gemini-api/docs/video-understanding)
- [Google Gemini API 文档](https://ai.google.dev/gemini-api/docs)
- [Google AI Studio](https://aistudio.google.com/)

## 许可证

请遵守 Gemini API 的使用条款和您的项目许可证。

## 联系支持

如有问题，请联系开发团队或查阅 Gemini API 官方文档。


