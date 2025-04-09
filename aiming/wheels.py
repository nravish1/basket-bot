import Math

#Method 1
#Takes input from a controller
#def mecanum_1():
    #speed = -gamepad1.left_stick_y
    #turn = gamepad1.right_stick_x
    #strafe = gamepad1.left_stick_x

    #leftFront = speed + turn + strafe
    #rightFront = speed - turn - strafe
    #leftRear = speed + turn - strafe
    #rightRear = speed - turn + strafe

#Method 2 - better than method 1
#theta is the direction of movement
#power is linear speed in direction of theta
#turn is rotational speed - clockwise or counterclockwise
def mecanum_2(theta, power, turn):
    if (hoop_detection):
        leftFront = 0
        rightFront = 0
        leftRear = 0
        rightRear = 0
    else:
        sin = Math.sin(theta - Math.PI/4)
        cos = Math.cos(theta - Math.PI/4)
        max = Math.max(Math.abs(sin), Math.abs(cos))

        leftFront = power * cos/max + turn
        rightFront = power * sin/max - turn
        leftRear = power * sin/max + turn
        rightRear = power * cos/max - turn

        if ((power + Math.abs(turn)) > 1):
            leftFront /= power + turn
            rightFront /= power + turn
            leftRear /= power + turn
            rightRear /= power + turn

#Computer vision functionality
def hoop_detection():
    #Return true when LiDAR sees the hoop
    return False

while not hoop_detection():
    mecanum_2(0, 0, 1)

mecanum_2(0, 0, 0)