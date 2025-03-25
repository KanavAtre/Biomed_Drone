import pyrealsense2 as rs
import numpy as np
import cv2

class RealSenseCapture:
    def __init__(self, width=1280, height=720, fps=30):
        self.width = width
        self.height = height
        self.fps = fps
        
        # Initialize RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Enable streams
        self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        
        self.align = rs.align(rs.stream.color)
        
    def start(self):
        try:
            profile = self.pipeline.start(self.config)
            
            # Get depth scale for distance calculations
            depth_sensor = profile.get_device().first_depth_sensor()
            self.depth_scale = depth_sensor.get_depth_scale()
            
            # Set high accuracy for depth sensor
            depth_sensor.set_option(rs.option.visual_preset, 3)
            
            return True
        except Exception as e:
            print(f"Failed to start RealSense pipeline: {e}")
            return False
            
    def get_frames(self):
        try:
            # Wait for a coherent pair of frames
            frames = self.pipeline.wait_for_frames()
            
            # Align the depth frame to color frame
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                return None, None
                
            # Convert frames to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data()).copy()
            color_image = np.asanyarray(color_frame.get_data()).copy()
            
            # Convert color image from BGR to grayscale for landing decision
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            
            return gray_image, depth_image
        except Exception as e:
            print(f"Failed to get frames: {e}")
            return None, None
            
    def get_sensor_data(self):
 
        sensor_data = {
            "wind_speed": 0,  
            "altitude": 100,  # This could potentially come from depth data average
            "obstacle_distance": self.get_min_distance()  # Using depth data for obstacle detection
        }
        return sensor_data
        
    def get_min_distance(self):
        try:
            frames = self.pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                return 100  # Default safe distance if no depth frame
                
            depth_image = np.asanyarray(depth_frame.get_data())
            min_distance = np.min(depth_image * self.depth_scale)
            return min_distance * 100  # Convert to centimeters
        except Exception:
            return 100  # Default safe distance on error
            
    def stop(self):
        """Stop the RealSense pipeline"""
        self.pipeline.stop() 
