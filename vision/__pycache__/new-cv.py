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
    head_angle_alpha = 0.25

    # Set up GPIO for servo
    SERVO_PIN = 18  # GPIO 18 (Pin 12)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(SERVO_PIN, GPIO.OUT)
    pwm = GPIO.PWM(SERVO_PIN, 50)  # 50Hz PWM for servo
    pwm.start(7.5)  # Center position

    def set_servo_angle(angle):
        duty = 2.5 + (angle / 180.0) * 10.0  # Map angle to duty cycle
        pwm.ChangeDutyCycle(duty)

    cap = cv2.VideoCapture(0)
    cap.set(3, 160)
    cap.set(4, 120)

    object_detector = cv2.createBackgroundSubtractorMOG2(history=10, varThreshold=5)

    try:
        while True:
            ret, frame = cap.read()
            height, width, _ = frame.shape
            roi = frame[0:240, 0:320]
            mask = object_detector.apply(roi)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            detections = []
            biggest_index = 0
            biggest_area = 0
            ind = 0
            for cnt in contours:
                area = cv2.contourArea(cnt)
                if area > 150:
                    x, y, w, h = cv2.boundingRect(cnt)
                    detections.append([x, y, w, h])
                    area = w * h
                    if area > biggest_area:
                        biggest_area = area
                        biggest_index = ind
                    ind += 1

            if len(detections) > 0:
                x, y, w, h = detections[biggest_index]
                cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 0), 3)
                head_angle = remap(float(x + (w / 2.0)), IN_MIN, IN_MAX, OUT_MIN, OUT_MAX)
                print('x: ' + str(x) + ', head: ' + str(head_angle))

            head_angle_ave = head_angle * head_angle_alpha + head_angle_ave * (1.0 - head_angle_alpha)
            set_servo_angle(head_angle_ave)

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