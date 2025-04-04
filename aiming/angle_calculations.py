import math

def launch_velocity(hoop_distance, angle):
    ramp_height = 0 #This is unknown
    g = 9.81
    hoop_height = 3.05
    #hoop_distance measured by LiDAR
    #angle will be calculated through trial and error

    theta = math.radians(angle)

    num_1 = g * (hoop_distance ** 2)
    num_2 = 2 * (hoop_distance * math.tan(theta) - (hoop_height - ramp_height)) * (math.cos(theta) ** 2)

    if ((num_1 / num_2) < 0):
        return 1000

    return math.sqrt(num_1 / num_2)

def ramp_angle(hoop_distance):
    theta = 0
    v0 = 1000

    for i in range(1, 90):
        j = launch_velocity(hoop_distance, i)
        if j < v0:
            v0 = j
            theta = i

    print(theta)
    print(v0)


ramp_angle(7.239) #3 point shot
ramp_angle(4.572) #Free throw line