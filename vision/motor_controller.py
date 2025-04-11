import math
MAX_SPEED = 255 # This will be changed based on how much our motors can handle

class MotorController:
    def __init__(self, max_speed=MAX_SPEED):
        self.max_speed = max_speed

    def simple_rotation(self, pid_output):
        pid_output = max(min(pid_output, self.max_speed), -self.max_speed)

        front_left_speed = -pid_output
        rear_left_speed	= -pid_output
        front_right_speed = pid_output
        rear_right_speed = pid_output

        return {
            "Front_Left" : front_left_speed,
            "Rear_Left" : rear_left_speed,
            "Front_Right" : front_right_speed,
            "Rear_Right" : rear_right_speed
        }

    # Anuj's Function Below (More Complicated Version) --- 
    # Theta and Power will stay at 0 for now (we might change this later based on needed level of complexity)
    # turn variable will be the PID output

    def mecanum_drive(self, theta, power, turn):
        sin = math.sin(theta - math.pi/4)
        cos = math.cos(theta - math.pi/4)
        max = max(abs(sin), abs(cos))

        leftFront = power * cos/max + turn
        rightFront = power * sin/max - turn
        leftRear = power * sin/max + turn
        rightRear = power * cos/max - turn

        if ((power + abs(turn)) > 1):
            leftFront /= power + turn
            rightFront /= power + turn
            leftRear /= power + turn
            rightRear /= power + turn

        return {
            "Front_Left" : leftFront,
            "Rear_Left" : leftRear,
            "Front_Right" : rightFront,
            "Rear_Right" : rightRear
        }

    def send_to_motors(self, speeds):
        print("Sending to motors:", speeds) # Placeholder function