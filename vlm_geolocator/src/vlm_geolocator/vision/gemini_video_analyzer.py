"""
Gemini API Video Analyzer
For analyzing casualties, trauma, and hemorrhage detection in drone footage
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
    print("Please install google-genai: pip install google-genai")
    raise


class TraumaLocation(Enum):
    """Trauma location enumeration"""
    HEAD = "Head"
    TRUNK = "Trunk/Torso"
    LIMBS = "Limbs"
    MULTIPLE = "Multiple"
    NONE = "None"


class HemorrhageSeverity(Enum):
    """Hemorrhage severity levels"""
    NONE = "No Hemorrhage"
    MINOR = "Minor Hemorrhage"
    MODERATE = "Moderate Hemorrhage"
    SEVERE = "Severe Hemorrhage"
    CRITICAL = "Critical Hemorrhage"


@dataclass
class CasualtyAnalysis:
    """Casualty analysis results"""
    is_casualty: bool
    confidence: float
    trauma_locations: List[TraumaLocation]
    hemorrhage_severity: HemorrhageSeverity
    detailed_findings: Dict[str, str]
    timestamps: List[str]
    raw_response: str


class GeminiVideoAnalyzer:
    """
    Gemini API-based video analyzer
    Specifically designed for detecting and assessing casualties in drone footage
    """
    
    # Detailed medical assessment prompt
    MEDICAL_ASSESSMENT_PROMPT = """
You are an experienced emergency medical response expert analyzing aerial footage from a drone.
The video may be shaky, but please do your best to analyze the personnel situation.

Please carefully analyze all persons in the video and conduct the following assessment for each person:

1. **Casualty Identification**:
   - Determine if the person in the frame is a casualty
   - Look for abnormal posture, lack of movement, or lying on the ground
   - Assess whether urgent medical assistance is needed

2. **Trauma Assessment** (if casualty detected):
   - Head Trauma: Is there visible injury, deformity, or abnormality to the head
   - Trunk Trauma: Are there visible signs of damage to chest or abdomen
   - Limb Trauma: Are arms or legs broken, deformed, or otherwise injured
   - **Amputation**: Look for missing limbs, severed body parts, or traumatic amputation
   - Record all visible trauma locations

3. **Hemorrhage Assessment**:
   - No Hemorrhage: No visible bleeding
   - Minor Hemorrhage: Small area bleeding, not life-threatening
   - Moderate Hemorrhage: Noticeable bleeding, requires treatment
   - Severe Hemorrhage: Large area bleeding, requires urgent hemostasis
   - Critical Hemorrhage: Life-threatening massive bleeding

4. **Respiratory Distress Assessment**:
   - Normal Breathing: No signs of respiratory distress
   - Labored Breathing: Visible difficulty breathing, chest heaving
   - Gasping: Irregular, desperate breathing attempts
   - No Breathing: No visible chest movement or respiratory effort
   - Look for: chest not moving, abnormal breathing patterns, cyanosis (blue/pale skin)

5. **Timestamp Marking**:
   - Mark time points of key findings (format: MM:SS)
   - Record time segments when casualties appear in frame

Please return the analysis results in structured JSON format:
{
  "casualties_detected": [
    {
      "casualty_id": 1,
      "is_casualty": true/false,
      "confidence_score": 0.0-1.0,
      "trauma_assessment": {
        "head": "description or 'none'",
        "trunk": "description or 'none'",
        "limbs": "description or 'none'",
        "amputation": "description of any missing/severed limbs or 'none'"
      },
      "hemorrhage": {
        "severity": "No Hemorrhage/Minor Hemorrhage/Moderate Hemorrhage/Severe Hemorrhage/Critical Hemorrhage",
        "location": "bleeding location description"
      },
      "respiratory_status": {
        "status": "Normal Breathing/Labored Breathing/Gasping/No Breathing/Unable to Assess",
        "observations": "detailed respiratory observations"
      },
      "position_status": "Standing/Walking/Lying/Other",
      "movement_observed": true/false,
      "timestamps": ["MM:SS", "MM:SS"],
      "detailed_observations": "detailed observation results"
    }
  ],
  "overall_assessment": "overall situation assessment",
  "recommended_actions": "recommended actions to take"
}

Notes:
- Even if the video is shaky, please try to analyze
- If uncertain, reflect the uncertainty in confidence_score
- Do NOT comment on video quality, lighting, resolution, or image clarity
- Do NOT mention footage limitations, blurriness, darkness, or pixelation
- Mark timestamps for all key moments
- If no casualties are found, please state clearly
- Pay special attention to amputation and respiratory distress signs as these are critical for triage
"""

    def __init__(self, api_key: Optional[str] = None, model: str = "gemini-2.5-flash"):
        """
        Initialize Gemini Video Analyzer
        
        Args:
            api_key: Gemini API key, if None will get from GEMINI_API_KEY environment variable
            model: Model name to use, default is gemini-2.5-flash
        """
        if api_key is None:
            api_key = os.environ.get("GEMINI_API_KEY")
            if api_key is None:
                raise ValueError(
                    "API key not found. Please set GEMINI_API_KEY environment variable or pass api_key parameter"
                )
        
        # Set API key
        os.environ["GEMINI_API_KEY"] = api_key
        
        self.client = genai.Client(api_key=api_key)
        self.model = model
        self.uploaded_files = {}  # Cache uploaded files
        
    def upload_video(self, video_path: str, force_reupload: bool = False) -> tuple:
        """
        Upload video file to Gemini
        
        Args:
            video_path: Video file path
            force_reupload: Whether to force reupload
            
        Returns:
            Uploaded file object
        """
        if not os.path.exists(video_path):
            raise FileNotFoundError(f"Video file not found: {video_path}")
        
        # Check cache
        if video_path in self.uploaded_files and not force_reupload:
            print(f"Using cached file: {video_path}")
            return self.uploaded_files[video_path]
        
        print(f"Uploading video: {video_path}")
        start_time = time.time()
        
        try:
            myfile = self.client.files.upload(file=video_path)
            upload_time = time.time() - start_time
            print(f"Video uploaded successfully! Time: {upload_time:.2f}s")
            print(f"File URI: {myfile.uri}")
            print(f"File name: {myfile.name}")
            
            # Wait for file processing
            print(f"Waiting for file processing (status: {myfile.state.name})...")
            while myfile.state.name == "PROCESSING":
                time.sleep(2)
                myfile = self.client.files.get(name=myfile.name)
                print(f"  Status: {myfile.state.name}")
            
            if myfile.state.name == "FAILED":
                raise ValueError(f"File processing failed: {myfile.name}")
            
            print(f"✅ File ready to use!")
            total_time = time.time() - start_time
            print(f"Total time: {total_time:.2f}s")
            
            # Cache file info
            self.uploaded_files[video_path] = myfile
            
            return myfile
            
        except Exception as e:
            print(f"Video upload failed: {e}")
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
        Analyze casualties in video
        
        Args:
            video_path: Video file path
            custom_prompt: Custom prompt (if not using default medical assessment prompt)
            fps: Custom frame rate sampling (default 1 FPS)
            start_offset: Start time offset (format: "40s" or "1:20")
            end_offset: End time offset
            
        Returns:
            CasualtyAnalysis: Casualty analysis results
        """
        # Upload video and wait for processing
        myfile = self.upload_video(video_path)
        
        # Prepare prompt
        prompt = custom_prompt if custom_prompt else self.MEDICAL_ASSESSMENT_PROMPT
        
        print("\nAnalyzing video...")
        print(f"Model: {self.model}")
        if fps:
            print(f"Sampling rate: {fps} FPS")
        if start_offset or end_offset:
            print(f"Analysis segment: {start_offset or 'start'} to {end_offset or 'end'}")
        
        try:
            # Build content
            parts = []
            
            # Add video part (with metadata)
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
            
            # Add text prompt
            parts.append(types.Part(text=prompt))
            
            # Generate content
            start_time = time.time()
            response = self.client.models.generate_content(
                model=self.model,
                contents=types.Content(parts=parts)
            )
            analysis_time = time.time() - start_time
            
            print(f"Analysis complete! Time: {analysis_time:.2f}s")
            
            # Parse response
            return self._parse_response(response.text)
            
        except Exception as e:
            print(f"Video analysis failed: {e}")
            raise
    
    def _parse_response(self, response_text: str) -> CasualtyAnalysis:
        """
        Parse Gemini response and extract structured information
        
        Args:
            response_text: Raw response text from Gemini API
            
        Returns:
            CasualtyAnalysis: Parsed casualty analysis results
        """
        print("\n=== Raw Response ===")
        print(response_text)
        print("=" * 50)
        
        # Try to extract JSON from response
        try:
            # Find JSON code block
            import re
            json_match = re.search(r'```json\s*(\{.*?\})\s*```', response_text, re.DOTALL)
            if json_match:
                json_str = json_match.group(1)
            else:
                # Try parsing entire response directly
                json_str = response_text
            
            data = json.loads(json_str)
            
            # Parse casualty information
            casualties = data.get('casualties_detected', [])
            
            if casualties and len(casualties) > 0:
                casualty = casualties[0]  # Take first casualty
                
                # Parse trauma locations
                trauma_locations = []
                trauma_assessment = casualty.get('trauma_assessment', {})
                
                # Check if head trauma exists (not "none" or contains "no")
                head_val = str(trauma_assessment.get('head', 'none')).lower()
                if head_val != 'none' and 'none' not in head_val and 'no ' not in head_val:
                    trauma_locations.append(TraumaLocation.HEAD)
                
                # Check if trunk trauma exists
                trunk_val = str(trauma_assessment.get('trunk', 'none')).lower()
                if trunk_val != 'none' and 'none' not in trunk_val and 'no ' not in trunk_val:
                    trauma_locations.append(TraumaLocation.TRUNK)
                
                # Check if limb trauma exists
                limbs_val = str(trauma_assessment.get('limbs', 'none')).lower()
                if limbs_val != 'none' and 'none' not in limbs_val and 'no ' not in limbs_val:
                    trauma_locations.append(TraumaLocation.LIMBS)
                
                if not trauma_locations:
                    trauma_locations.append(TraumaLocation.NONE)
                
                # Parse hemorrhage severity
                hemorrhage_text = casualty.get('hemorrhage', {}).get('severity', 'No Hemorrhage')
                hemorrhage_map = {
                    'No Hemorrhage': HemorrhageSeverity.NONE,
                    'Minor Hemorrhage': HemorrhageSeverity.MINOR,
                    'Moderate Hemorrhage': HemorrhageSeverity.MODERATE,
                    'Severe Hemorrhage': HemorrhageSeverity.SEVERE,
                    'Critical Hemorrhage': HemorrhageSeverity.CRITICAL,
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
                # No casualties detected
                return CasualtyAnalysis(
                    is_casualty=False,
                    confidence=0.9,
                    trauma_locations=[TraumaLocation.NONE],
                    hemorrhage_severity=HemorrhageSeverity.NONE,
                    detailed_findings={
                        'overall': data.get('overall_assessment', 'No casualties detected'),
                        'recommendations': data.get('recommended_actions', '')
                    },
                    timestamps=[],
                    raw_response=response_text
                )
                
        except json.JSONDecodeError:
            # Cannot parse JSON, return basic info
            print("Warning: Cannot parse as JSON format, returning raw text")
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
        Print analysis results (formatted output)
        
        Args:
            analysis: Casualty analysis results
        """
        print("\n" + "=" * 60)
        print("Casualty Analysis Results")
        print("=" * 60)
        
        print(f"\nIs Casualty: {'Yes' if analysis.is_casualty else 'No'}")
        print(f"Confidence: {analysis.confidence:.2%}")
        
        if analysis.is_casualty:
            print(f"\nTrauma Locations:")
            for location in analysis.trauma_locations:
                print(f"  - {location.value}")
            
            print(f"\nHemorrhage Severity: {analysis.hemorrhage_severity.value}")
            
            if analysis.timestamps:
                print(f"\nKey Timestamps:")
                for ts in analysis.timestamps:
                    print(f"  - {ts}")
        
        print(f"\nDetailed Findings:")
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
    Main function - example usage
    """
    import sys
    
    # Check command line arguments
    if len(sys.argv) < 2:
        print("Usage: python gemini_video_analyzer.py <video_path> [API_key]")
        print("\nOr set environment variable GEMINI_API_KEY")
        sys.exit(1)
    
    video_path = sys.argv[1]
    api_key = sys.argv[2] if len(sys.argv) > 2 else None
    
    try:
        # Create analyzer
        analyzer = GeminiVideoAnalyzer(api_key=api_key)
        
        # Analyze video
        # For shaky drone videos, can lower fps to reduce processing time
        analysis = analyzer.analyze_video(
            video_path=video_path,
            fps=1.0  # 1 frame per second
        )
        
        # Print results
        analyzer.print_analysis(analysis)
        
        # Save results to JSON
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
        print(f"Analysis results saved to: {output_path}")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)


if __name__ == "__main__":
    main()

