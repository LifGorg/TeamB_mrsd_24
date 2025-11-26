"""Configuration Management Module"""
import yaml
from pathlib import Path
from typing import Dict, Any
from dataclasses import dataclass
import numpy as np


@dataclass
class CameraConfig:
    """Camera Configuration"""
    name: str
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    # Gimbal attitude - NO hardcoded defaults, must come from camera_config.yaml
    gimbal_roll_deg: float
    gimbal_pitch_deg: float
    gimbal_yaw_deg: float
    
    @property
    def intrinsic_matrix(self) -> np.ndarray:
        """Return camera intrinsic matrix"""
        return np.array([
            [self.fx, 0, self.cx],
            [0, self.fy, self.cy],
            [0, 0, 1]
        ])
    
    @property
    def gimbal_attitude_radians(self) -> tuple:
        """Return gimbal attitude in radians (roll, pitch, yaw)"""
        return (
            np.radians(self.gimbal_roll_deg),
            np.radians(self.gimbal_pitch_deg),
            np.radians(self.gimbal_yaw_deg)
        )


@dataclass
class GStreamerConfig:
    """GStreamer Configuration"""
    mode: str  # "auto" or "custom"
    
    # Auto mode parameters (required fields)
    udp_port: int
    encoding: str
    
    # Network/jitter side parameters (required fields)
    udpsrc_buffer_size: int  # Buffer size for udpsrc (bytes)
    jitterbuffer_latency: int  # Latency for rtpjitterbuffer (milliseconds)
    jitterbuffer_drop_on_latency: bool  # Drop packets that arrive too late
    
    # Pipeline latency / backpressure parameters (required fields)
    queue_max_size_time: int  # Max time in nanoseconds
    queue_leaky: str  # Leak type: "no", "upstream", or "downstream"
    
    # App-side buffering parameters (required fields)
    appsink_max_buffers: int  # Maximum buffers to queue in appsink
    appsink_drop: bool  # Drop old buffers when queue is full
    
    # Optional parameters (with defaults)
    multicast_enabled: bool = False
    multicast_address: str = ""
    multicast_interface: str = ""
    
    # Legacy parameters (deprecated, kept for backward compatibility)
    buffer_size: int = 0
    latency: int = 0
    max_buffers: int = 0
    drop: bool = True
    
    # Custom mode parameter
    custom_pipeline: str = ""
    
    def get_pipeline_string(self) -> str:
        """Generate GStreamer pipeline string
        
        Returns:
            - custom_pipeline if mode="custom"
            - auto-generated pipeline if mode="auto"
        """
        # Custom mode: use provided pipeline string
        if self.mode == "custom":
            if not self.custom_pipeline or not self.custom_pipeline.strip():
                raise ValueError(
                    "GStreamer mode is set to 'custom' but custom_pipeline is empty. "
                    "Either provide a custom_pipeline or set mode to 'auto'."
                )
            return self.custom_pipeline.strip()
        
        # Auto mode: generate pipeline from parameters
        if self.mode != "auto":
            raise ValueError(f"Invalid GStreamer mode: '{self.mode}'. Must be 'auto' or 'custom'.")
        
        # Decoder based on encoding
        decoder = 'rtph265depay ! h265parse ! avdec_h265' if self.encoding == 'H265' else 'rtph264depay ! h264parse ! avdec_h264'
        
        # Build udpsrc with multicast support if enabled
        if self.multicast_enabled:
            udpsrc = f"udpsrc address={self.multicast_address} port={self.udp_port} multicast-iface={self.multicast_interface} buffer-size={self.udpsrc_buffer_size} auto-multicast=true"
        else:
            udpsrc = f"udpsrc port={self.udp_port} buffer-size={self.udpsrc_buffer_size}"
        
        # Build rtpjitterbuffer with configurable parameters
        drop_on_latency_str = "true" if self.jitterbuffer_drop_on_latency else "false"
        jitterbuffer = f"rtpjitterbuffer latency={self.jitterbuffer_latency} drop-on-latency={drop_on_latency_str}"
        
        # Build queue with configurable parameters
        queue = f"queue max-size-time={self.queue_max_size_time} leaky={self.queue_leaky}"
        
        # Build appsink with configurable parameters
        appsink_drop_str = "true" if self.appsink_drop else "false"
        appsink = f"appsink name=appsink0 emit-signals=true max-buffers={self.appsink_max_buffers} drop={appsink_drop_str}"
        
        # Single-line pipeline (no newlines)
        pipeline = f"{udpsrc} ! application/x-rtp,media=video,clock-rate=90000,encoding-name={self.encoding},payload=96 ! {jitterbuffer} ! {decoder} ! {queue} ! videoconvert ! video/x-raw,format=BGR ! {appsink}"
        return pipeline


@dataclass
class ROS2Config:
    """ROS2 Configuration"""
    input_topics: Dict[str, str]
    output_topics: Dict[str, str]
    services: Dict[str, str]
    qos: Dict[str, Any]


@dataclass
class SystemConfig:
    """System Configuration"""
    sensor_timeout: float
    sensor_stale_warning: float
    warning_interval: float
    startup_grace_period: float
    log_dir: str
    model_path: str
    model_device: str
    model_dtype: str
    gps_earth_radius_lat: float
    gps_snap_to_reference: bool
    gps_snap_noise_meters: float  # Legacy, for backward compatibility
    gps_gaussian_sigma_meters: float
    gps_uniform_range_meters: float
    thread_pool_max_workers: int
    debug_display_enabled: bool
    debug_display_interval: int
    debug_display_window_name: str
    video_recording_duration: float
    video_recording_min_fps: float


class ConfigManager:
    """Configuration Manager"""
    
    def __init__(self, config_dir: str = None):
        if config_dir is None:
            # Default config directory in project root
            config_dir = Path(__file__).parent.parent.parent.parent / "config"
        else:
            config_dir = Path(config_dir)
        
        self.config_dir = config_dir
        self._load_all_configs()
    
    def _load_yaml(self, filename: str) -> Dict[str, Any]:
        """Load YAML file"""
        filepath = self.config_dir / filename
        if not filepath.exists():
            raise FileNotFoundError(f"Configuration file does not exist: {filepath}")
        
        with open(filepath, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f)
    
    def _load_all_configs(self):
        """Load all configurations"""
        # Load camera configuration
        camera_data = self._load_yaml("camera_config.yaml")
        cam_cfg = camera_data['camera']
        
        # Load gimbal attitude - REQUIRED, no hardcoded defaults
        gimbal_attitude = cam_cfg.get('gimbal_attitude')
        if gimbal_attitude is None:
            raise ValueError(
                "Missing 'gimbal_attitude' in camera_config.yaml. "
                "This configuration is required and must include roll, pitch, and yaw values."
            )
        
        # Verify all required gimbal attitude fields are present
        required_fields = ['roll', 'pitch', 'yaw']
        missing_fields = [f for f in required_fields if f not in gimbal_attitude]
        if missing_fields:
            raise ValueError(
                f"Missing gimbal_attitude fields in camera_config.yaml: {', '.join(missing_fields)}. "
                f"Required fields: {', '.join(required_fields)}"
            )
        
        self.camera = CameraConfig(
            name=cam_cfg['name'],
            width=cam_cfg['resolution']['width'],
            height=cam_cfg['resolution']['height'],
            fx=cam_cfg['intrinsics']['fx'],
            fy=cam_cfg['intrinsics']['fy'],
            cx=cam_cfg['intrinsics']['cx'],
            cy=cam_cfg['intrinsics']['cy'],
            gimbal_roll_deg=gimbal_attitude['roll'],
            gimbal_pitch_deg=gimbal_attitude['pitch'],
            gimbal_yaw_deg=gimbal_attitude['yaw']
        )
        
        # Load GStreamer configuration
        gst_data = self._load_yaml("system_config.yaml")['gstreamer']
        mode = gst_data.get('mode', 'auto')
        
        # Validate mode
        if mode not in ['auto', 'custom']:
            raise ValueError(
                f"Invalid GStreamer mode '{mode}' in camera_config.yaml. "
                "Must be 'auto' or 'custom'."
            )
        
        # Support both new and legacy parameter names
        # New parameters take precedence over legacy ones
        udpsrc_buffer_size = gst_data.get('udpsrc_buffer_size', gst_data.get('buffer_size', 4194304))
        jitterbuffer_latency = gst_data.get('jitterbuffer_latency', gst_data.get('latency', 600))
        appsink_max_buffers = gst_data.get('appsink_max_buffers', gst_data.get('max_buffers', 10))
        appsink_drop = gst_data.get('appsink_drop', gst_data.get('drop', True))
        
        self.gstreamer = GStreamerConfig(
            mode=mode,
            udp_port=gst_data.get('udp_port', 5000),
            encoding=gst_data.get('encoding', 'H264'),
            multicast_enabled=gst_data.get('multicast_enabled', False),
            multicast_address=gst_data.get('multicast_address', ''),
            multicast_interface=gst_data.get('multicast_interface', ''),
            # Network/jitter side parameters
            udpsrc_buffer_size=udpsrc_buffer_size,
            jitterbuffer_latency=jitterbuffer_latency,
            jitterbuffer_drop_on_latency=gst_data.get('jitterbuffer_drop_on_latency', True),
            # Pipeline latency / backpressure parameters
            queue_max_size_time=gst_data.get('queue_max_size_time', 300000000),
            queue_leaky=gst_data.get('queue_leaky', 'downstream'),
            # App-side buffering parameters
            appsink_max_buffers=appsink_max_buffers,
            appsink_drop=appsink_drop,
            # Legacy parameters (for backward compatibility)
            buffer_size=gst_data.get('buffer_size', 0),
            latency=gst_data.get('latency', 0),
            max_buffers=gst_data.get('max_buffers', 0),
            drop=gst_data.get('drop', True),
            custom_pipeline=gst_data.get('custom_pipeline', '')
        )
        
        # Load ROS2 configuration
        ros_data = self._load_yaml("ros_config.yaml")['ros2']
        self.ros2 = ROS2Config(
            input_topics=ros_data['input_topics'],
            output_topics=ros_data['output_topics'],
            services=ros_data['services'],
            qos=ros_data['qos']
        )
        
        # Load system configuration
        system_data = self._load_yaml("system_config.yaml")['system']
        debug_display = system_data.get('debug_display', {})
        video_recording = system_data.get('video_recording', {})
        
        # Load GPS configuration for snap noise and snap_to_reference
        gps_data = self._load_yaml("gps_config.yaml")['gps']
        snap_to_reference = gps_data.get('snap_to_reference', True)  # Default True if not specified
        
        # Load noise configuration (supports both new and legacy formats)
        snap_noise_config = gps_data.get('snap_noise', {})
        if isinstance(snap_noise_config, dict):
            # New format: separate Gaussian and Uniform parameters
            gaussian_sigma = snap_noise_config.get('gaussian_sigma_meters', 0.3)
            uniform_range = snap_noise_config.get('uniform_range_meters', 0.5)
            snap_noise_legacy = snap_noise_config.get('snap_noise_meters', 0.0)
        else:
            # Legacy format: single snap_noise_meters value
            snap_noise_legacy = gps_data.get('snap_noise_meters', 0.8)
            gaussian_sigma = 0.3  # Default
            uniform_range = snap_noise_legacy if snap_noise_legacy > 0 else 0.5
        
        self.system = SystemConfig(
            sensor_timeout=system_data['sensor_timeout'],
            sensor_stale_warning=system_data['sensor_stale_warning'],
            warning_interval=system_data['warning_interval'],
            startup_grace_period=system_data['startup_grace_period'],
            log_dir=system_data['log_dir'],
            model_path=system_data['model']['path'],
            model_device=system_data['model']['device'],
            model_dtype=system_data['model']['dtype'],
            gps_earth_radius_lat=system_data['gps']['earth_radius_lat'],
            gps_snap_to_reference=snap_to_reference,
            gps_snap_noise_meters=snap_noise_legacy,
            gps_gaussian_sigma_meters=gaussian_sigma,
            gps_uniform_range_meters=uniform_range,
            thread_pool_max_workers=system_data['thread_pool']['max_workers'],
            debug_display_enabled=debug_display.get('enabled', False),
            debug_display_interval=debug_display.get('display_interval', 30),
            debug_display_window_name=debug_display.get('window_name', 'Video Feed'),
            video_recording_duration=video_recording.get('duration', 5.0),
            video_recording_min_fps=video_recording.get('min_fps', 2.0)
        )
    
    def get_camera_config(self) -> CameraConfig:
        """Get camera configuration"""
        return self.camera
    
    def get_gstreamer_config(self) -> GStreamerConfig:
        """Get GStreamer configuration"""
        return self.gstreamer
    
    def get_ros2_config(self) -> ROS2Config:
        """Get ROS2 configuration"""
        return self.ros2
    
    def get_system_config(self) -> SystemConfig:
        """Get system configuration"""
        return self.system
