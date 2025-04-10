import time
from hoop_detection import HoopDetector

class PIDController:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.prev_error = 0 # To calculate the derivative
        self.integral = 0 # To sum up the errors

    def update(self, error, dt):
        self.proportional = self.Kp * error
        self.integral += error * dt
        self.derivative = (error - self.prev_error) / dt
        output = self.proportional + self.Ki * self.integral + self.Kd * self.derivative
        self.prev_error = error

        return output


    