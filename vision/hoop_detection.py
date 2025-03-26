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
        # self.cap = cv2.VideoCapture(0)

        # Testing w/ Videos
        # self.cap = cv2.VideoCapture("vision/video_test_5.mp4")
        self.cap = cv2.VideoCapture("vision/vecteezy_business-man-in-suit-and-tie-plays-basketball_3208542.mp4")
        
        self.hoop_pos = [] # List to store hoop positions
        self.frame_count = 0 # Frame count for the video
        self.frame = None # Current frame from the video

        self.run() # Start the detection loop

    def run(self):
        while True:
            ret, self.frame = self.cap.read() # Read a frame from the video
            if not ret:
                # End of the video or an error occurred
                break

            results = self.model(self.frame, stream=True, device=self.device) # Perform inference on the current frame

            for r in results:
                boxes = r.boxes
                for box in boxes:
                    # Bounding box
                    x1, y1, x2, y2 = box.xyxy[0]
                    x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                    w, h = x2 - x1, y2 - y1

                    # Confidence
                    conf = math.ceil((box.conf[0] * 100)) / 100

                    # Class Name
                    cls = int(box.cls[0])
                    current_class = self.class_names[cls]

                    center = (int(x1 + w / 2), int(y1 + h / 2))

                    # Create hoop points if high confidence
                    if conf > .5 and current_class == "Basketball Hoop":
                        self.hoop_pos.append((center, self.frame_count, w, h, conf))
                        cvzone.cornerRect(self.frame, (x1, y1, w, h))

            self.frame_count += 1
            cv2.imshow('Frame', self.frame) # Display the current frame

            # Close if 'q' is clicked
            if cv2.waitKey(1) & 0xFF == ord('q'):  # higher waitKey slows video down, use 1 for webcam
                break

        self.cap.release() # Release the video capture object (stops the camera)
        cv2.destroyAllWindows() # Close all OpenCV windows

if __name__ == "__main__":
    HoopDetector()

