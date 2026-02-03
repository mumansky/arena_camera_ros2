#!/usr/bin/env python3
"""
Simple polarized image viewer for ROS2
Subscribes to polarized camera topic and displays split channels
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class PolarizedViewer(Node):
    def __init__(self):
        super().__init__('polarized_viewer')
        
        self.bridge = CvBridge()
        
        # Use best_effort QoS to match the camera publisher
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # Subscribe to the raw polarized image topic
        self.subscription = self.create_subscription(
            Image,
            '/arena_camera_node/images',
            self.image_callback,
            qos)
        
        self.get_logger().info('Polarized Viewer started - subscribing to /arena_camera_node/images')
        
        # Windows for display
        cv2.namedWindow('Polarized 0°', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Polarized 45°', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Polarized 90°', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Polarized 135°', cv2.WINDOW_NORMAL)
        
    def image_callback(self, msg):
        """Process incoming polarized image"""
        try:
            # Handle the custom polarized_angles encoding
            if 'polarized_angles' in msg.encoding.lower():
                # Raw data from PolarizedAngles_0d_45d_90d_135d_BayerRG8
                width = msg.width
                height = msg.height
                
                # Work with actual data size (camera may be compressing or sending incomplete frames)
                raw_data = np.frombuffer(msg.data, dtype=np.uint8)
                actual_size = len(raw_data)
                
                self.get_logger().info(f'Received {width}x{height}, data size: {actual_size}', 
                                      throttle_duration_sec=2.0)
                
                # For now, just display the raw data as a grayscale image to see what we're getting
                # Calculate dimensions that fit the actual data
                if actual_size > 0:
                    # Try interpreting as a single-channel image
                    pixels = actual_size
                    est_height = int(np.sqrt(pixels / (width / height)))
                    est_width = int(pixels / est_height)
                    
                    if est_height > 0 and est_width > 0 and est_height * est_width <= actual_size:
                        img = raw_data[:est_height * est_width].reshape((est_height, est_width))
                        cv2.imshow('Raw Data', img)
                        cv2.waitKey(1)
                    else:
                        self.get_logger().warn(f'Cannot reshape {actual_size} bytes', 
                                             throttle_duration_sec=2.0)
                    
            # Handle standard ROS encodings            
            elif msg.encoding == 'rgba8' or msg.encoding == '8UC4':
                # 4-channel interleaved format
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                
                if cv_image.shape[2] == 4:
                    # Split into 4 channels: [0°, 45°, 90°, 135°]
                    p0 = cv_image[:, :, 0]
                    p45 = cv_image[:, :, 1]
                    p90 = cv_image[:, :, 2]
                    p135 = cv_image[:, :, 3]
                    
                    # Each channel is Bayer-encoded, demosaic to color
                    # PHX050S1-Q uses BayerRG pattern
                    p0_color = cv2.cvtColor(p0, cv2.COLOR_BayerRG2BGR)
                    p45_color = cv2.cvtColor(p45, cv2.COLOR_BayerRG2BGR)
                    p90_color = cv2.cvtColor(p90, cv2.COLOR_BayerRG2BGR)
                    p135_color = cv2.cvtColor(p135, cv2.COLOR_BayerRG2BGR)
                    
                    # Display all 4 polarization angles
                    cv2.imshow('Polarized 0°', p0_color)
                    cv2.imshow('Polarized 45°', p45_color)
                    cv2.imshow('Polarized 90°', p90_color)
                    cv2.imshow('Polarized 135°', p135_color)
                    
                    cv2.waitKey(1)
                else:
                    self.get_logger().warn(f'Expected 4 channels, got {cv_image.shape[2]}')
            else:
                self.get_logger().warn(f'Unexpected encoding: {msg.encoding}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    viewer = PolarizedViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        viewer.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
