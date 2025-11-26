"""
Gemini API 视频分析器
用于分析无人机视频中的伤员情况、创伤和出血检测
"""

import os
import json
import time
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

try:
    from google import genai
    from google.genai import types
except ImportError:
    print("请安装 google-genai: pip install google-genai")
    raise


class TraumaLocation(Enum):
    """创伤位置枚举"""
    HEAD = "头部"
    TRUNK = "躯干"
    LIMBS = "四肢"
    MULTIPLE = "多处"
    NONE = "无"


class HemorrhageSeverity(Enum):
    """出血严重程度"""
    NONE = "无出血"
    MINOR = "轻微出血"
    MODERATE = "中度出血"
    SEVERE = "严重出血"
    CRITICAL = "危急出血"


@dataclass
class CasualtyAnalysis:
    """伤员分析结果"""
    is_casualty: bool
    confidence: float
    trauma_locations: List[TraumaLocation]
    hemorrhage_severity: HemorrhageSeverity
    detailed_findings: Dict[str, str]
    timestamps: List[str]
    raw_response: str


class GeminiVideoAnalyzer:
    """
    基于 Gemini API 的视频分析器
    专门用于检测和评估无人机视频中的伤员情况
    """
    
    # 详细的医疗评估提示词
    MEDICAL_ASSESSMENT_PROMPT = """
你是一名经验丰富的紧急医疗响应专家，正在分析来自无人机的航拍视频。
视频可能有抖动，但请尽力分析其中的人员情况。

请仔细分析视频中的所有人员，并针对每个人进行以下评估：

1. **伤员识别**：
   - 判断画面中的人是否为伤员（casualty）
   - 寻找异常姿势、缺乏移动、或躺倒在地等迹象
   - 评估是否需要紧急医疗援助

2. **创伤评估**（如发现伤员）：
   - 头部创伤：头部是否有明显伤口、变形或异常
   - 躯干创伤：胸部或腹部是否有明显损伤迹象
   - 四肢创伤：手臂或腿部是否有断裂、变形或其他损伤
   - 记录所有可见的创伤位置

3. **出血评估**：
   - 无出血：没有可见出血
   - 轻微出血：小面积出血，不危及生命
   - 中度出血：明显出血，需要处理
   - 严重出血：大面积出血，需要紧急止血
   - 危急出血：威胁生命的大量出血

4. **时间戳标记**：
   - 标记关键发现的时间点（格式：MM:SS）
   - 记录伤员在画面中出现的时间段

请以结构化的JSON格式返回分析结果：
{
  "casualties_detected": [
    {
      "casualty_id": 1,
      "is_casualty": true/false,
      "confidence_score": 0.0-1.0,
      "trauma_assessment": {
        "head": "描述或'无'",
        "trunk": "描述或'无'",
        "limbs": "描述或'无'"
      },
      "hemorrhage": {
        "severity": "无出血/轻微出血/中度出血/严重出血/危急出血",
        "location": "出血位置描述"
      },
      "position_status": "站立/行走/躺卧/其他",
      "movement_observed": true/false,
      "timestamps": ["MM:SS", "MM:SS"],
      "detailed_observations": "详细观察结果"
    }
  ],
  "overall_assessment": "整体情况评估",
  "recommended_actions": "建议采取的行动",
  "video_quality_notes": "视频质量备注（如抖动、模糊等）"
}

注意：
- 即使视频抖动，也请尽力分析
- 如果不确定，请在confidence_score中反映不确定性
- 标注所有关键时刻的时间戳
- 如果没有发现伤员，请明确说明
"""

    def __init__(self, api_key: Optional[str] = None, model: str = "gemini-2.5-flash"):
        """
        初始化 Gemini 视频分析器
        
        Args:
            api_key: Gemini API 密钥，如果为None则从环境变量GEMINI_API_KEY获取
            model: 使用的模型名称，默认为 gemini-2.5-flash
        """
        if api_key is None:
            api_key = os.environ.get("GEMINI_API_KEY")
            if api_key is None:
                raise ValueError(
                    "未找到API密钥。请设置GEMINI_API_KEY环境变量或通过参数传递api_key"
                )
        
        # 设置API密钥
        os.environ["GEMINI_API_KEY"] = api_key
        
        self.client = genai.Client(api_key=api_key)
        self.model = model
        self.uploaded_files = {}  # 缓存已上传的文件
        
    def upload_video(self, video_path: str, force_reupload: bool = False) -> tuple:
        """
        上传视频文件到 Gemini
        
        Args:
            video_path: 视频文件路径
            force_reupload: 是否强制重新上传
            
        Returns:
            上传后的文件对象
        """
        if not os.path.exists(video_path):
            raise FileNotFoundError(f"视频文件不存在: {video_path}")
        
        # 检查缓存
        if video_path in self.uploaded_files and not force_reupload:
            print(f"使用缓存的文件: {video_path}")
            return self.uploaded_files[video_path]
        
        print(f"正在上传视频: {video_path}")
        start_time = time.time()
        
        try:
            myfile = self.client.files.upload(file=video_path)
            upload_time = time.time() - start_time
            print(f"视频上传成功! 耗时: {upload_time:.2f}秒")
            print(f"文件URI: {myfile.uri}")
            print(f"文件名: {myfile.name}")
            
            # 等待文件处理完成
            print(f"等待文件处理中 (状态: {myfile.state.name})...")
            while myfile.state.name == "PROCESSING":
                time.sleep(2)
                myfile = self.client.files.get(name=myfile.name)
                print(f"  状态: {myfile.state.name}")
            
            if myfile.state.name == "FAILED":
                raise ValueError(f"文件处理失败: {myfile.name}")
            
            print(f"✅ 文件已就绪，可以使用!")
            total_time = time.time() - start_time
            print(f"总耗时: {total_time:.2f}秒")
            
            # 缓存文件信息
            self.uploaded_files[video_path] = myfile
            
            return myfile
            
        except Exception as e:
            print(f"视频上传失败: {e}")
            raise
    
    def analyze_video(
        self,
        video_path: str,
        custom_prompt: Optional[str] = None,
        fps: Optional[float] = None,
        start_offset: Optional[str] = None,
        end_offset: Optional[str] = None
    ) -> CasualtyAnalysis:
        """
        分析视频中的伤员情况
        
        Args:
            video_path: 视频文件路径
            custom_prompt: 自定义提示词（如果不想使用默认的医疗评估提示）
            fps: 自定义帧率采样（默认1 FPS）
            start_offset: 开始时间偏移（格式："40s" 或 "1:20"）
            end_offset: 结束时间偏移
            
        Returns:
            CasualtyAnalysis: 伤员分析结果
        """
        # 上传视频并等待处理
        myfile = self.upload_video(video_path)
        
        # 准备提示词
        prompt = custom_prompt if custom_prompt else self.MEDICAL_ASSESSMENT_PROMPT
        
        print("\n正在进行视频分析...")
        print(f"模型: {self.model}")
        if fps:
            print(f"采样率: {fps} FPS")
        if start_offset or end_offset:
            print(f"分析片段: {start_offset or '开始'} 到 {end_offset or '结束'}")
        
        try:
            # 构建内容
            parts = []
            
            # 添加视频部分（带元数据）
            video_metadata = {}
            if fps:
                video_metadata['fps'] = fps
            if start_offset:
                video_metadata['start_offset'] = start_offset
            if end_offset:
                video_metadata['end_offset'] = end_offset
            
            if video_metadata:
                parts.append(
                    types.Part(
                        file_data=types.FileData(file_uri=myfile.uri),
                        video_metadata=types.VideoMetadata(**video_metadata)
                    )
                )
            else:
                parts.append(types.Part(file_data=types.FileData(file_uri=myfile.uri)))
            
            # 添加文本提示
            parts.append(types.Part(text=prompt))
            
            # 生成内容
            start_time = time.time()
            response = self.client.models.generate_content(
                model=self.model,
                contents=types.Content(parts=parts)
            )
            analysis_time = time.time() - start_time
            
            print(f"分析完成! 耗时: {analysis_time:.2f}秒")
            
            # 解析响应
            return self._parse_response(response.text)
            
        except Exception as e:
            print(f"视频分析失败: {e}")
            raise
    
    def _parse_response(self, response_text: str) -> CasualtyAnalysis:
        """
        解析 Gemini 响应并提取结构化信息
        
        Args:
            response_text: Gemini API 的原始响应文本
            
        Returns:
            CasualtyAnalysis: 解析后的伤员分析结果
        """
        print("\n=== 原始响应 ===")
        print(response_text)
        print("=" * 50)
        
        # 尝试从响应中提取JSON
        try:
            # 寻找JSON代码块
            import re
            json_match = re.search(r'```json\s*(\{.*?\})\s*```', response_text, re.DOTALL)
            if json_match:
                json_str = json_match.group(1)
            else:
                # 尝试直接解析整个响应
                json_str = response_text
            
            data = json.loads(json_str)
            
            # 解析伤员信息
            casualties = data.get('casualties_detected', [])
            
            if casualties and len(casualties) > 0:
                casualty = casualties[0]  # 取第一个伤员
                
                # 解析创伤位置
                trauma_locations = []
                trauma_assessment = casualty.get('trauma_assessment', {})
                if trauma_assessment.get('head', '无').lower() != '无':
                    trauma_locations.append(TraumaLocation.HEAD)
                if trauma_assessment.get('trunk', '无').lower() != '无':
                    trauma_locations.append(TraumaLocation.TRUNK)
                if trauma_assessment.get('limbs', '无').lower() != '无':
                    trauma_locations.append(TraumaLocation.LIMBS)
                if not trauma_locations:
                    trauma_locations.append(TraumaLocation.NONE)
                
                # 解析出血严重程度
                hemorrhage_text = casualty.get('hemorrhage', {}).get('severity', '无出血')
                hemorrhage_map = {
                    '无出血': HemorrhageSeverity.NONE,
                    '轻微出血': HemorrhageSeverity.MINOR,
                    '中度出血': HemorrhageSeverity.MODERATE,
                    '严重出血': HemorrhageSeverity.SEVERE,
                    '危急出血': HemorrhageSeverity.CRITICAL,
                }
                hemorrhage_severity = hemorrhage_map.get(hemorrhage_text, HemorrhageSeverity.NONE)
                
                return CasualtyAnalysis(
                    is_casualty=casualty.get('is_casualty', False),
                    confidence=casualty.get('confidence_score', 0.0),
                    trauma_locations=trauma_locations,
                    hemorrhage_severity=hemorrhage_severity,
                    detailed_findings={
                        'trauma': trauma_assessment,
                        'hemorrhage': casualty.get('hemorrhage', {}),
                        'position': casualty.get('position_status', ''),
                        'movement': casualty.get('movement_observed', False),
                        'observations': casualty.get('detailed_observations', ''),
                        'overall': data.get('overall_assessment', ''),
                        'recommendations': data.get('recommended_actions', '')
                    },
                    timestamps=casualty.get('timestamps', []),
                    raw_response=response_text
                )
            else:
                # 没有检测到伤员
                return CasualtyAnalysis(
                    is_casualty=False,
                    confidence=0.9,
                    trauma_locations=[TraumaLocation.NONE],
                    hemorrhage_severity=HemorrhageSeverity.NONE,
                    detailed_findings={
                        'overall': data.get('overall_assessment', '未检测到伤员'),
                        'recommendations': data.get('recommended_actions', '')
                    },
                    timestamps=[],
                    raw_response=response_text
                )
                
        except json.JSONDecodeError:
            # 无法解析JSON，返回基本信息
            print("警告: 无法解析为JSON格式，返回原始文本")
            return CasualtyAnalysis(
                is_casualty=False,
                confidence=0.0,
                trauma_locations=[TraumaLocation.NONE],
                hemorrhage_severity=HemorrhageSeverity.NONE,
                detailed_findings={'raw_text': response_text},
                timestamps=[],
                raw_response=response_text
            )
    
    def print_analysis(self, analysis: CasualtyAnalysis):
        """
        打印分析结果（格式化输出）
        
        Args:
            analysis: 伤员分析结果
        """
        print("\n" + "=" * 60)
        print("伤员分析结果")
        print("=" * 60)
        
        print(f"\n是否为伤员: {'是' if analysis.is_casualty else '否'}")
        print(f"置信度: {analysis.confidence:.2%}")
        
        if analysis.is_casualty:
            print(f"\n创伤位置:")
            for location in analysis.trauma_locations:
                print(f"  - {location.value}")
            
            print(f"\n出血严重程度: {analysis.hemorrhage_severity.value}")
            
            if analysis.timestamps:
                print(f"\n关键时间点:")
                for ts in analysis.timestamps:
                    print(f"  - {ts}")
        
        print(f"\n详细发现:")
        for key, value in analysis.detailed_findings.items():
            if isinstance(value, dict):
                print(f"  {key}:")
                for k, v in value.items():
                    print(f"    {k}: {v}")
            else:
                print(f"  {key}: {value}")
        
        print("=" * 60 + "\n")


def main():
    """
    主函数 - 示例用法
    """
    import sys
    
    # 检查命令行参数
    if len(sys.argv) < 2:
        print("用法: python gemini_video_analyzer.py <视频路径> [API密钥]")
        print("\n或设置环境变量 GEMINI_API_KEY")
        sys.exit(1)
    
    video_path = sys.argv[1]
    api_key = sys.argv[2] if len(sys.argv) > 2 else None
    
    try:
        # 创建分析器
        analyzer = GeminiVideoAnalyzer(api_key=api_key)
        
        # 分析视频
        # 对于抖动的无人机视频，可以降低fps以减少处理时间
        analysis = analyzer.analyze_video(
            video_path=video_path,
            fps=1.0  # 每秒1帧
        )
        
        # 打印结果
        analyzer.print_analysis(analysis)
        
        # 保存结果到JSON
        output_path = video_path.replace('.mp4', '_analysis.json')
        with open(output_path, 'w', encoding='utf-8') as f:
            json.dump({
                'is_casualty': analysis.is_casualty,
                'confidence': analysis.confidence,
                'trauma_locations': [loc.value for loc in analysis.trauma_locations],
                'hemorrhage_severity': analysis.hemorrhage_severity.value,
                'detailed_findings': analysis.detailed_findings,
                'timestamps': analysis.timestamps,
                'raw_response': analysis.raw_response
            }, f, ensure_ascii=False, indent=2)
        print(f"分析结果已保存到: {output_path}")
        
    except Exception as e:
        print(f"错误: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

