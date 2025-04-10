from hoop_detection import HoopDetector
from pid_controller import PIDController
import time

if __name__ == "__main__":
    detector = HoopDetector() # Initialize the hoop detector
    pid = PIDController(Kp=0.01, Ki=0.0001, Kd=0.005) # Initialize PID with starting values. Tune them later.
    prev_time = time.time()

    while True:
        success = detector.process_frame()
        if not success:
            break

        current_time = time.time()
        dt = current_time - prev_time
        prev_time = current_time

        error = detector.get_horizontal_error()
        if error is not None:
            output = pid.update(error, dt)
            detector.pid_error = error
            detector.pid_output = output
            print(f"Error: {error:.2f}, PID Output: {output:.2f}")
        else:
            print("Hoop not detected — skipping PID update.")
    
