import time
from lsm6dsox import LSM6DSOX
from machine import Pin
from machine import SPI

# Initialize sensor
lsm = LSM6DSOX(SPI(5), cs=Pin("PF6", Pin.OUT_PP, Pin.PULL_UP))

def classify_motion(ax, ay, az):
    # Calculate the magnitude of the acceleration vector
    magnitude = (ax**2 + ay**2 + az**2) ** 0.5

    # Threshold values for different states
    idle_threshold = 0.5
    walk_threshold = 1.5
    run_threshold = 3.0
    shake_threshold = 2.0

    # Motion classification logic
    if magnitude < idle_threshold:
        return "걷기 (Walking)"
    elif magnitude < walk_threshold:
        return "가만히 있기 (IDLE)"
    elif magnitude < run_threshold:
        return "뛰기 (Running)"
    else:
        return "털기 (Shaking)"

while True:
    # Read accelerometer data
    ax, ay, az = lsm.accel()

    # Classify the current motion
    state = classify_motion(ax, ay, az)

    # Print the results
    print("Accelerometer: x:{:>8.3f} y:{:>8.3f} z:{:>8.3f} - 상태: {}".format(ax, ay, az, state))
    print("")

    time.sleep_ms(100)
