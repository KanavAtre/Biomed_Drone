from realsense_capture import RealSenseCapture
from landing_pipeline import LandingDecisionPipeline
import cv2
import numpy as np

def main():
    camera = RealSenseCapture(width=1280, height=720, fps=30)
    if not camera.start():
        print("Failed to start RealSense camera")
        return

    landing_pipeline = LandingDecisionPipeline(landing_threshold=0.7)

    try:
        while True:
            gray_image, depth_image = camera.get_frames()
            if gray_image is None or depth_image is None:
                print("Failed to get frames")
                continue

            sensor_data = camera.get_sensor_data()

            # Make landing decision
            can_land = landing_pipeline.decide_landing(gray_image, sensor_data)

            # Display results
            # Create a visualization window
            cv2.imshow('Gray Image', gray_image)
            
            depth_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
            depth_normalized = np.uint8(depth_normalized)
            cv2.imshow('Depth Image', depth_normalized)

            status = "SAFE TO LAND" if can_land else "UNSAFE TO LAND"
            color = (0, 255, 0) if can_land else (0, 0, 255)
            cv2.putText(gray_image, status, (30, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        camera.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main() 
