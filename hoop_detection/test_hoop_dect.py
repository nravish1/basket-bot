import cv2
import numpy as np

# Load the image
image_path = "C:/Users/Allana/Dev/Ram Robotics/basketbotRepo3_7/basket-bot/hoop_detection/basketball-hoop.jpg"
image = cv2.imread(image_path)

# Check if image was loaded successfully
if image is None:
    print("Error loading image.")
    exit()

# Step 1: Convert the image to grayscale
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Step 2: Apply Gaussian blur to reduce noise and improve circle detection
blurred = cv2.GaussianBlur(gray, (15, 15), 0)

# Step 3: Use Hough Circle Transform to detect circles (hoops)
circles = cv2.HoughCircles(blurred, 
                            cv2.HOUGH_GRADIENT, dp=1.2, minDist=100, 
                            param1=50, param2=30, minRadius=50, maxRadius=100)

# Step 4: If circles are found, draw them on the original image
if circles is not None:
    circles = np.round(circles[0, :]).astype("int")
    for (x, y, r) in circles:
        # Draw the outer circle (hoop)
        cv2.circle(image, (x, y), r, (0, 255, 0), 4)
        # Draw the center of the circle
        cv2.circle(image, (x, y), 2, (0, 0, 255), 3)
    
    print(f"Detected {len(circles)} hoop(s).")
else:
    print("No hoops detected.")

# Step 5: Display the result
cv2.imshow("Basketball Hoop Detection", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
