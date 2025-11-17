# HTML Viewer Improvements - Medical Triage Display

## 概述
优化了 Gemini 结果查看器的 HTML 显示，提供清晰、紧凑的医疗分类信息，供医疗人员快速决策。

## 修改内容

### 1. 更新 Gemini 提示词 (`gemini_video_analyzer.py`)

**新增评估项目：**
- **截肢评估 (Amputation Assessment)**：检测缺失肢体、断肢或创伤性截肢
- **呼吸窘迫评估 (Respiratory Distress Assessment)**：
  - Normal Breathing（正常呼吸）
  - Labored Breathing（费力呼吸）
  - Gasping（喘息）
  - No Breathing（无呼吸）
  - 检查胸部运动、呼吸模式异常、紫绀等

**更新的 JSON 结构：**
```json
{
  "trauma_assessment": {
    "head": "...",
    "trunk": "...",
    "limbs": "...",
    "amputation": "description of any missing/severed limbs or 'none'"
  },
  "respiratory_status": {
    "status": "Normal Breathing/Labored Breathing/Gasping/No Breathing/Unable to Assess",
    "observations": "detailed respiratory observations"
  }
}
```

### 2. 新的 HTML 显示格式 (`gemini_result_viewer_html.py`)

#### 紧凑医疗信息卡片
显示 7 个关键医疗指标，采用 2x3 网格布局：

1. **Casualty Status（伤员状态）**
   - YES/NO 显示
   - 置信度百分比

2. **GPS Coordinates（GPS 坐标）**
   - 从文件名提取（casualty_LAT_LON 格式）

3. **Trauma Assessment（创伤评估）**
   - 创伤位置数量
   - 具体位置列表

4. **Hemorrhage Severity（出血严重程度）**
   - 5 级分类（None/Minor/Moderate/Severe/Critical）

5. **Amputation Status（截肢状态）**
   - 检测缺失肢体
   - 从 trauma_assessment.amputation 字段提取

6. **Respiratory Status（呼吸状态）**
   - 从 respiratory_status 字段提取
   - 回退到关键词检测（observations）

7. **Triage Level（分级级别）**
   - 大横幅显示：RED/YELLOW/GREEN/BLACK
   - 基于所有医疗指标自动计算

#### 分级算法 (Triage Level Calculation)

**RED（红色 - 立即救治）：**
- 呼吸停止或喘息
- 严重或危急出血
- 创伤性截肢

**YELLOW（黄色 - 延迟救治）：**
- 中度出血
- 费力呼吸
- 多处创伤（≥2 处）

**GREEN（绿色 - 轻伤）：**
- 轻微出血
- 单处创伤
- 能行走的伤员

### 3. 视觉改进

**颜色编码：**
- 红色边框：伤员状态、出血
- 蓝色边框：GPS、呼吸
- 橙色边框：截肢
- 紫色/粉色边框：创伤

**分级横幅：**
```
┌────────────────────────────────┐
│   TRIAGE LEVEL: RED/YELLOW/GREEN   │
└────────────────────────────────┘
```
- RED：深红背景 + 红色边框
- YELLOW：深黄背景 + 黄色边框
- GREEN：深绿背景 + 绿色边框

**信息卡片：**
- 统一样式的紧凑卡片
- 标题（灰色小字）+ 值（白色粗体）
- 左侧彩色边框指示器

### 4. 后向兼容性

代码支持新旧两种 JSON 格式：

**新格式（推荐）：**
```json
{
  "trauma_assessment": {
    "amputation": "left leg severed below knee"
  },
  "respiratory_status": {
    "status": "No Breathing",
    "observations": "..."
  }
}
```

**旧格式（回退）：**
- 从 `detailed_findings.observations` 中检测关键词
- 从 trauma 描述中查找 "amput", "missing limb", "severed"

## 使用方法

### 查看 HTML
HTML 文件自动生成在：
```
/home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator/gemini_analysis_results.html
```

在浏览器中打开即可查看。

### 监视模式（Watch Mode）
```bash
cd /home/triage/georgia_dtc_ops_team_chiron_mrsd/vlm_geolocator
python3 gemini_result_viewer_html.py --watch --interval 5
```

自动监测新视频并更新 HTML。

### 测试新提示词
下次录制新视频时，Gemini 会使用更新后的提示词进行分析，自动提取截肢和呼吸状态信息。

## 医疗人员快速使用指南

1. **查看分级横幅**：立即识别优先级（RED/YELLOW/GREEN）
2. **扫描 6 个信息卡片**：
   - 左上：伤员状态
   - 右上：GPS 位置
   - 中左：创伤
   - 中右：出血
   - 下左：截肢
   - 下右：呼吸
3. **阅读详细观察**：滚动到下方查看完整分析
4. **查看推荐行动**：Gemini 生成的救治建议

## 优势

✅ **紧凑清晰**：所有关键信息一屏显示  
✅ **视觉编码**：颜色快速识别危急情况  
✅ **自动分级**：基于多指标智能计算  
✅ **实时更新**：Watch 模式持续监控  
✅ **医疗标准**：遵循战地分类原则（RED/YELLOW/GREEN）

## 下一步改进建议

1. **添加时间戳显示**：显示首次发现时间
2. **地图集成**：点击 GPS 在地图上标注
3. **导出 PDF 报告**：生成医疗报告文档
4. **多伤员对比**：并排显示多个伤员
5. **BLACK 级别**：添加"期待死亡"分类（目前未使用）



