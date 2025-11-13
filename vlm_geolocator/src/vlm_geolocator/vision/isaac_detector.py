"""Isaac-0.1 Detector Wrapper"""
import torch
import re
import json
from pathlib import Path
from PIL import Image
from datetime import datetime
from typing import List, Tuple
from transformers import AutoConfig, AutoModelForCausalLM, AutoProcessor
import numpy as np

from .detector_interface import DetectorInterface


class IsaacDetectorWrapper(DetectorInterface):
    """Isaac-0.1 Detector"""
    
    def __init__(
        self,
        model_path: str,
        device: str = None,
        save_dir: str = None
    ):
        """
        Initialize detector
        
        Args:
            model_path: Model path
            device: Device ('cuda', 'cpu' or None for auto-detection)
            save_dir: Directory path for saving inputs/outputs
        """
        self.model_path = Path(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(f"Model does not exist: {self.model_path}")
        
        # Device configuration
        if device is None:
            self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        else:
            self.device = torch.device(device)
        
        # Save directory
        if save_dir:
            self.save_dir = Path(save_dir)
            self.save_dir.mkdir(parents=True, exist_ok=True)
        else:
            self.save_dir = None
        
        # Load model
        self._load_model()
    
    def _load_model(self):
        """Load model components"""
        self.config = AutoConfig.from_pretrained(
            str(self.model_path), trust_remote_code=True
        )
        self.processor = AutoProcessor.from_pretrained(
            str(self.model_path), trust_remote_code=True
        )
        self.model = AutoModelForCausalLM.from_pretrained(
            str(self.model_path), trust_remote_code=True
        )
        
        # Move to device
        if self.device.type == "cuda":
            self.model = self.model.to(self.device, dtype=torch.bfloat16)
        else:
            self.model = self.model.to(self.device, dtype=torch.float32)
        
        self.model.eval()
    
    def detect(self, image: np.ndarray) -> List[Tuple[float, float]]:
        """
        Detect humans in image
        
        Args:
            image: numpy array image
            
        Returns:
            Detection results [(cx, cy), ...]
        """
        # Convert to PIL Image
        if not isinstance(image, Image.Image):
            image = Image.fromarray(image)
        
        image_width, image_height = image.size
        
        # Save input image (if enabled)
        if self.save_dir:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            input_image_path = self.save_dir / f"{timestamp}_input.jpg"
            image.save(input_image_path, quality=95)
        
        # Prepare input
        messages = [
            {"role": "user", "content": "<hint>BOX</hint>"},
            {"role": "user", "content": self.config.vision_token},
            {"role": "user", "content": "point people on images"}
        ]
        
        text_fmt = self.processor.apply_chat_template(
            messages, tokenize=False, add_generation_prompt=True
        )
        inputs = self.processor(text=text_fmt, images=[image], return_tensors="pt")
        
        # Run inference
        with torch.no_grad():
            tensor_stream = inputs["tensor_stream"].to(self.device)
            generated_ids = self.model.generate(
                tensor_stream=tensor_stream,
                max_new_tokens=256,
                do_sample=False,
                eos_token_id=self.processor.tokenizer.eos_token_id
            )
        
        # Decode output
        response_text = self.processor.tokenizer.decode(
            generated_ids[0], skip_special_tokens=True
        )
        
        # Parse results
        results = self._parse_detections(response_text, image_width, image_height)
        
        # Save results (if enabled)
        if self.save_dir:
            output_log = {
                "timestamp": timestamp,
                "image_size": {"width": image_width, "height": image_height},
                "model_raw_output": response_text,
                "parsed_detections": [
                    {"center": (float(cx), float(cy))} for cx, cy in results
                ],
                "detection_count": len(results)
            }
            
            output_log_path = self.save_dir / f"{timestamp}_output.json"
            with open(output_log_path, 'w', encoding='utf-8') as f:
                json.dump(output_log, f, indent=2, ensure_ascii=False)
        
        return results
    
    def _parse_detections(
        self,
        response: str,
        image_width: int,
        image_height: int
    ) -> List[Tuple[float, float]]:
        """Parse model output"""
        # Match <point_box> (x1,y1) (x2,y2) </point_box>
        pattern = r'<point_box[^>]*>\s*\((\d+),(\d+)\)\s*\((\d+),(\d+)\)\s*</point_box>'
        matches = re.findall(pattern, response)
        
        results = []
        for match in matches:
            x1_norm, y1_norm, x2_norm, y2_norm = map(int, match)
            
            # Convert coordinates: from 0-1000 normalized values to pixel coordinates
            x1 = (x1_norm / 1000.0) * image_width
            y1 = (y1_norm / 1000.0) * image_height
            x2 = (x2_norm / 1000.0) * image_width
            y2 = (y2_norm / 1000.0) * image_height
            
            # Calculate center point
            cx = (x1 + x2) / 2.0
            cy = (y1 + y2) / 2.0
            
            results.append((cx, cy))
        
        return results
