#!/usr/bin/env python3
"""
Casualty Email Notifier Node

Listens to /casualty_geolocated topic and sends email notifications 
whenever a new casualty location is detected.

Usage:
    python3 casualty_email_notifier.py

Configuration:
    Edit email_config.yaml to set SMTP settings and recipient list
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from datetime import datetime
import yaml
from pathlib import Path
from typing import List, Dict
import threading


class CasualtyEmailNotifier(Node):
    """ROS2 Node that sends email notifications for detected casualties"""
    
    def __init__(self):
        super().__init__('casualty_email_notifier')
        
        # Load email configuration
        self.config = self._load_config()
        
        # Email sending queue and lock
        self.email_lock = threading.Lock()
        self.last_notification_time = None
        self.notification_count = 0
        
        # Subscribe to casualty geolocated topic
        self.subscription = self.create_subscription(
            NavSatFix,
            '/casualty_geolocated',
            self._casualty_callback,
            10
        )
        
        self.get_logger().info('📧 Casualty Email Notifier started')
        self.get_logger().info(f'   SMTP Server: {self.config["smtp"]["server"]}:{self.config["smtp"]["port"]}')
        self.get_logger().info(f'   Recipients: {", ".join(self.config["recipients"])}')
        self.get_logger().info(f'   From: {self.config["sender"]["email"]}')
        self.get_logger().info('   Listening to: /casualty_geolocated')
        
    def _load_config(self) -> Dict:
        """Load email configuration from YAML file"""
        config_path = Path(__file__).parent.parent / 'config' / 'email_config.yaml'
        
        if config_path.exists():
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
                self.get_logger().info(f'✅ Loaded config from: {config_path}')
                return config
        else:
            # Default configuration
            self.get_logger().warn(f'⚠️  Config file not found: {config_path}')
            self.get_logger().warn('   Using default configuration')
            return {
                'smtp': {
                    'server': 'smtp.gmail.com',
                    'port': 587,
                    'use_tls': True,
                    'username': 'your-email@gmail.com',
                    'password': 'your-app-password'  # Use app-specific password for Gmail
                },
                'sender': {
                    'email': 'your-email@gmail.com',
                    'name': 'Chiron Casualty Detection System'
                },
                'recipients': [
                    'recipient1@example.com',
                    'recipient2@example.com'
                ],
                'email_template': {
                    'subject': '🚨 CASUALTY DETECTED - {timestamp}',
                    'priority': 'high'
                }
            }
    
    def _casualty_callback(self, msg: NavSatFix):
        """
        Callback when new casualty location is published
        
        Args:
            msg: NavSatFix message with casualty GPS coordinates
        """
        self.notification_count += 1
        timestamp = datetime.now()
        
        self.get_logger().info(
            f'📍 Casualty #{self.notification_count} detected: '
            f'lat={msg.latitude:.6f}, lon={msg.longitude:.6f}, alt={msg.altitude:.1f}m'
        )
        
        # Send email notification in a separate thread to avoid blocking
        threading.Thread(
            target=self._send_email_notification,
            args=(msg, timestamp),
            daemon=True
        ).start()
    
    def _send_email_notification(self, msg: NavSatFix, timestamp: datetime):
        """
        Send email notification about detected casualty
        
        Args:
            msg: NavSatFix message with casualty GPS coordinates
            timestamp: Detection timestamp
        """
        with self.email_lock:
            try:
                # Create email message
                email_msg = self._create_email_message(msg, timestamp)
                
                # Send email
                self._send_email(email_msg)
                
                self.last_notification_time = timestamp
                self.get_logger().info(
                    f'✅ Email notification #{self.notification_count} sent successfully'
                )
                
            except Exception as e:
                self.get_logger().error(f'❌ Failed to send email: {e}')
    
    def _create_email_message(self, msg: NavSatFix, timestamp: datetime) -> MIMEMultipart:
        """
        Create formatted email message
        
        Args:
            msg: NavSatFix message with casualty GPS coordinates
            timestamp: Detection timestamp
            
        Returns:
            MIMEMultipart email message
        """
        # Create message container
        email_msg = MIMEMultipart('alternative')
        
        # Subject
        subject = self.config['email_template']['subject'].format(
            timestamp=timestamp.strftime('%Y-%m-%d %H:%M:%S')
        )
        email_msg['Subject'] = subject
        email_msg['From'] = f"{self.config['sender']['name']} <{self.config['sender']['email']}>"
        email_msg['To'] = ', '.join(self.config['recipients'])
        
        # Set priority
        if self.config['email_template'].get('priority') == 'high':
            email_msg['X-Priority'] = '1'
            email_msg['Importance'] = 'high'
        
        # Google Maps link
        maps_link = f"https://www.google.com/maps?q={msg.latitude},{msg.longitude}"
        
        # Create plain text version
        text_content = f"""
CASUALTY DETECTION ALERT

Time: {timestamp.strftime('%Y-%m-%d %H:%M:%S')}
Casualty Number: #{self.notification_count}

GPS COORDINATES:
  Latitude:  {msg.latitude:.6f}°
  Longitude: {msg.longitude:.6f}°
  Altitude:  {msg.altitude:.1f} meters

Google Maps: {maps_link}

STATUS:
  GPS Status: {msg.status.status}
  Position Covariance Type: {msg.position_covariance_type}

---
This is an automated message from Chiron Casualty Detection System.
Please respond immediately with appropriate medical assistance.
        """
        
        # Create HTML version
        html_content = f"""
<!DOCTYPE html>
<html>
<head>
    <style>
        body {{
            font-family: Arial, sans-serif;
            background-color: #f5f5f5;
            margin: 0;
            padding: 20px;
        }}
        .container {{
            max-width: 600px;
            margin: 0 auto;
            background-color: white;
            border-radius: 10px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
            overflow: hidden;
        }}
        .header {{
            background: linear-gradient(135deg, #ff4444 0%, #cc0000 100%);
            color: white;
            padding: 30px;
            text-align: center;
        }}
        .header h1 {{
            margin: 0;
            font-size: 24px;
        }}
        .content {{
            padding: 30px;
        }}
        .info-box {{
            background-color: #f9f9f9;
            border-left: 4px solid #ff4444;
            padding: 15px;
            margin: 20px 0;
        }}
        .info-label {{
            font-weight: bold;
            color: #666;
            font-size: 12px;
            text-transform: uppercase;
        }}
        .info-value {{
            font-size: 18px;
            color: #333;
            margin-top: 5px;
        }}
        .gps-coords {{
            background-color: #e8f4ff;
            border-left: 4px solid #0066cc;
            padding: 20px;
            margin: 20px 0;
            border-radius: 5px;
        }}
        .map-button {{
            display: inline-block;
            background-color: #0066cc;
            color: white;
            padding: 15px 30px;
            text-decoration: none;
            border-radius: 5px;
            font-weight: bold;
            margin-top: 20px;
        }}
        .map-button:hover {{
            background-color: #0052a3;
        }}
        .footer {{
            background-color: #f9f9f9;
            padding: 20px;
            text-align: center;
            font-size: 12px;
            color: #666;
            border-top: 1px solid #eee;
        }}
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>🚨 CASUALTY DETECTED</h1>
        </div>
        
        <div class="content">
            <div class="info-box">
                <div class="info-label">Detection Time</div>
                <div class="info-value">{timestamp.strftime('%Y-%m-%d %H:%M:%S')}</div>
            </div>
            
            <div class="info-box">
                <div class="info-label">Casualty Number</div>
                <div class="info-value">#{self.notification_count}</div>
            </div>
            
            <div class="gps-coords">
                <h3 style="margin-top: 0; color: #0066cc;">📍 GPS Coordinates</h3>
                <table style="width: 100%;">
                    <tr>
                        <td style="padding: 8px;"><strong>Latitude:</strong></td>
                        <td style="padding: 8px; text-align: right; font-family: monospace;">{msg.latitude:.6f}°</td>
                    </tr>
                    <tr>
                        <td style="padding: 8px;"><strong>Longitude:</strong></td>
                        <td style="padding: 8px; text-align: right; font-family: monospace;">{msg.longitude:.6f}°</td>
                    </tr>
                    <tr>
                        <td style="padding: 8px;"><strong>Altitude:</strong></td>
                        <td style="padding: 8px; text-align: right; font-family: monospace;">{msg.altitude:.1f} m</td>
                    </tr>
                </table>
                
                <a href="{maps_link}" class="map-button" target="_blank">
                    🗺️ Open in Google Maps
                </a>
            </div>
            
            <div style="background-color: #fff3cd; border-left: 4px solid #ffc107; padding: 15px; margin: 20px 0;">
                <strong>⚠️ ACTION REQUIRED</strong>
                <p style="margin: 10px 0 0 0;">Please respond immediately with appropriate medical assistance to the location above.</p>
            </div>
        </div>
        
        <div class="footer">
            This is an automated message from <strong>Chiron Casualty Detection System</strong>.<br>
            Generated by Vision-Language Model Geolocator on Domain 100.
        </div>
    </div>
</body>
</html>
        """
        
        # Attach both versions
        part1 = MIMEText(text_content, 'plain')
        part2 = MIMEText(html_content, 'html')
        
        email_msg.attach(part1)
        email_msg.attach(part2)
        
        return email_msg
    
    def _send_email(self, email_msg: MIMEMultipart):
        """
        Send email via SMTP
        
        Args:
            email_msg: Formatted email message to send
        """
        smtp_config = self.config['smtp']
        
        # Connect to SMTP server
        if smtp_config['use_tls']:
            server = smtplib.SMTP(smtp_config['server'], smtp_config['port'])
            server.starttls()
        else:
            server = smtplib.SMTP_SSL(smtp_config['server'], smtp_config['port'])
        
        try:
            # Login
            server.login(smtp_config['username'], smtp_config['password'])
            
            # Send email
            server.send_message(email_msg)
            
        finally:
            server.quit()


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    node = CasualtyEmailNotifier()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('📧 Shutting down email notifier...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

