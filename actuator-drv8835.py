from pololu_drv8835_rpi import motors, MAX_SPEED

# throttle
cur_speed = 0
cur_steer = 0

# init
def init(default_speed=50):
    motors.setSpeeds(0, 0)

def set_throttle(throttle_pct):
    global cur_speed
    cur_speed = throttle_pct
    motors.motor2.setSpeed(cur_speed)

def set_steering(steering_deg):
    if steering_deg < -10:
        left()
    elif steering_deg > 10:
        right()
    else:
        center()

# steering
def center():
    motors.motor1.setSpeed(0)
def left():
    motors.motor1.setSpeed(MAX_SPEED)
def right():
    motors.motor1.setSpeed(-MAX_SPEED)

# exit    
def turn_off():
    set_throttle(0)
    center()
