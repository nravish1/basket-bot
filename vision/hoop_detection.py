from ultralytics import YOLO
import cv2
import cvzone
import math
import numpy as np
from utils import get_device

class HoopDetector:
    def __init__(self):
        self.overlay_text = "Waiting..."
        self.model = YOLO("vision/best.pt") 
        self.class_names = ['Basketball', 'Basketball Hoop']
        self.device = get_device() # This will need to be changed to Raspberry Pi CPU
        
        # Uncomment line below to use the camera
        self.cap = cv2.VideoCapture(0)

        # Testing w/ Videos
        # self.cap = cv2.VideoCapture("vision/video_test_5.mp4")
        
        self.hoop_pos = [] # List to store hoop positions
        self.frame_count = 0 # Frame count for the video
        self.frame = None # Frame is null initially

        self.hoop_center = None # Initialize hoop center
        self.frame_center = None # Initialize frame center

        self.pid_error = None
        self.pid_output = None

    def process_frame(self):
        ret, self.frame = self.cap.read() # Read a frame from the video
        if not ret:
            return False # End of the video or an error occurred

        self.frame_center = self.draw_center_frame() # Draw a circle at the center of the frame
        results = self.model(self.frame, stream=True, device=self.device) # Perform inference on the current frame
        self.hoop_center = None # Initialize hoop center

        for r in results:
            boxes = r.boxes
            for box in boxes:
                # Bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                w, h = x2 - x1, y2 - y1

                conf = math.ceil((box.conf[0] * 100)) / 100 # Confidence

                cls = int(box.cls[0])
                current_class = self.class_names[cls]
                center_box = (int(x1 + w / 2), int(y1 + h / 2)) # Center of the bounding box

                # Create hoop points if high confidence
                if conf > .5 and current_class == "Basketball Hoop":
                    self.hoop_center = center_box
                    self.hoop_pos.append((center_box, self.frame_count, w, h, conf))
                    cvzone.cornerRect(self.frame, (x1, y1, w, h))

        if self.hoop_center:
            self.distance_hoop_center(self.frame_center, self.hoop_center) # Calculate the distance from the frame center to the hoop center

        self.frame_count += 1

        if self.pid_error is not None:
            cv2.putText(self.frame, f"PID Error: {self.pid_error:.2f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        if self.pid_output is not None:
            cv2.putText(self.frame, f"PID Output: {self.pid_output:.2f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
        cv2.imshow('Frame', self.frame) # Display the current frame

        # Close if 'q' is clicked
        if cv2.waitKey(1) & 0xFF == ord('q'):  # higher waitKey slows video down, use 1 for webcam
            return False
        
        return True

    def draw_center_frame(self):
        """Draw a circle at the center of the frame"""
        center = (int(self.frame.shape[1] / 2), int(self.frame.shape[0] / 2))
        cv2.circle(self.frame, center, 5, (0, 255, 0), -1)
        return center

    def distance_hoop_center(self, frame_center, hoop_center):
        """Calculate the distance between the center of the frame and the hoop center"""
        distance = math.sqrt((frame_center[0] - hoop_center[0]) ** 2 + (frame_center[1] - hoop_center[1]) ** 2)
        cv2.line(self.frame, frame_center, hoop_center, (0, 0, 255), 2)
        cv2.putText(self.frame, f"Dist: {distance}px", (frame_center[0] + 10, frame_center[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        print(f"Distance to hoop center: {distance}px")
        
    def get_horizontal_error(self):
        """Return horizontal distance (X-axis) between hoop center and frame center"""
        if self.hoop_center and self.frame_center:
            return self.frame_center[0] - self.hoop_center[0]
        
        return None
    
    def shutdown(self):
        self.cap.release()
        cv2.destroyAllWindows()

    def get_latest_frame(self):
        return self.frame