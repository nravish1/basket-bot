import numpy as np
import cv2
import RPi.GPIO as GPIO
import time
import sys
import datetime

def main():
    print('SKELLINGTON ALIVE!')
    print(datetime.datetime.now())

    IN_MIN = 30.0
    IN_MAX = 160.0
    OUT_MIN = 0.0    # Min servo angle
    OUT_MAX = 180.0  # Max servo angle

    head_angle = 90.0
    head_angle_ave = 90.0
    head_angle_alpha = 0.25  # Smoothing factor for the servo angle

    # Set up GPIO for servo
    SERVO_PIN = 18  # GPIO 18 (Pin 12)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM for servo
    pwm.start(7.5)  # Center position

    def set_servo_angle(angle):
        duty = 2.5 + (angle / 180.0) * 10.0  # Map angle to duty cycle
        pwm.ChangeDutyCycle(duty)

    # initialize the HOG descriptor/person detector
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    cap = cv2.VideoCapture(0)
    cap.set(3, 160)
    cap.set(4, 120)

    try:
        while True:
            ret, frame = cap.read()

            # resizing for faster detection
            frame = cv2.resize(frame, (640, 480))

            # detect people in the image (use grayscale for faster processing)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # detect humans in the frame
            boxes, weights = hog.detectMultiScale(frame, winStride=(8, 8))

            boxes = np.array([[x, y, x + w, y + h] for (x, y, w, h) in boxes])

            if len(boxes) > 0:
                # Assuming the largest box corresponds to the closest person
                xA, yA, xB, yB = max(boxes, key=lambda b: (b[2] - b[0]) * (b[3] - b[1]))

                # Draw the rectangle on the frame
                cv2.rectangle(frame, (xA, yA), (xB, yB), (0, 255, 0), 2)

                # Calculate the head angle based on the detected human's position
                center_x = float(xA + (xB - xA) / 2.0)
                head_angle = remap(center_x, 0, frame.shape[1], OUT_MIN, OUT_MAX)
                print('Center X: ' + str(center_x) + ', Head Angle: ' + str(head_angle))

                # Apply smoothing to the head angle
                head_angle_ave = head_angle * head_angle_alpha + head_angle_ave * (1.0 - head_angle_alpha)
                set_servo_angle(head_angle_ave)

            # Display the resulting frame
            cv2.imshow('frame', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Exiting...")

    finally:
        pwm.stop()
        GPIO.cleanup()
        cap.release()
        cv2.destroyAllWindows()

def remap(x, in_min, in_max, out_min, out_max):
    x_diff = x - in_min
    out_range = out_max - out_min
    in_range = in_max - in_min
    temp_out = x_diff * out_range / in_range + out_min

    if out_max < out_min:
        out_max, out_min = out_min, out_max

    return max(min(temp_out, out_max), out_min)

if __name__ == "__main__":
    main()