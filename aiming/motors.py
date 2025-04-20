import time
import pigpio

# Define GPIO pins for both ESCs
ESC1_GPIO = 18  # First flywheel motor (e.g., left)
ESC2_GPIO = 19  # Second flywheel motor (e.g., right)

# Create pigpio instance
pi = pigpio.pi()
if not pi.connected:
    print("Failed to connect to pigpio daemon. Exiting.")
    exit()

# ESC PWM signal range in microseconds
ESC_MIN = 1000  # Minimum throttle (1.0ms pulse)
ESC_MAX = 2000  # Maximum throttle (2.0ms pulse)

def set_esc_pulsewidth(gpio, us):
    """Set pulse width in microseconds for ESC on given GPIO pin."""
    pi.set_servo_pulsewidth(gpio, us)

def set_throttle(gpio, throttle):
    """
    Set throttle from 0.0 to 1.0 on a specific ESC.
    """
    throttle = max(0.0, min(1.0, throttle))  # Clamp to valid range
    pulse = ESC_MIN + (ESC_MAX - ESC_MIN) * throttle
    set_esc_pulsewidth(gpio, pulse)

def arm_escs():
    """Send zero throttle to both ESCs to arm them."""
    print("Arming ESCs...")
    set_throttle(ESC1_GPIO, 0.0)
    set_throttle(ESC2_GPIO, 0.0)
    time.sleep(2)  # Give ESCs time to beep and arm

try:
    print("Starting flywheel motors setup...")
    arm_escs()

    print("Running both flywheels at 50% throttle for 5 seconds...")
    set_throttle(ESC1_GPIO, 0.5)
    set_throttle(ESC2_GPIO, 0.5)
    time.sleep(5)

    print("Running both flywheels at 100% throttle for 3 seconds...")
    set_throttle(ESC1_GPIO, 1.0)
    set_throttle(ESC2_GPIO, 1.0)
    time.sleep(3)

    print("Stopping both motors...")
    set_throttle(ESC1_GPIO, 0.0)
    set_throttle(ESC2_GPIO, 0.0)
    time.sleep(2)

except KeyboardInterrupt:
    print("Interrupted! Stopping flywheels...")
    set_throttle(ESC1_GPIO, 0.0)
    set_throttle(ESC2_GPIO, 0.0)

finally:
    # Fully release PWM signals
    pi.set_servo_pulsewidth(ESC1_GPIO, 0)
    pi.set_servo_pulsewidth(ESC2_GPIO, 0)
    pi.stop()
    print("ESCs shut down. pigpio daemon stopped.")