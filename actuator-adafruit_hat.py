from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor

# init
mh = Adafruit_MotorHAT(addr=0x60)

# throttle
cur_speed = 0
MAX_SPEED = 255

# Actuator API:
#   init, set_throttle, set_steering

def init(default_speed=0):
    global steering, throttle
    steering = mh.getMotor(1)
    throttle = mh.getMotor(2)
    set_throttle(default_speed)
    
def set_throttle(throttle_pct):
    global cur_speed
    speed = int(MAX_SPEED * throttle_pct / 100)
    cur_speed = min(MAX_SPEED, speed)
    print ("speed: %d" % cur_speed)

    if throttle_pct > 0:
        throttle.setSpeed(cur_speed)
        throttle.run(Adafruit_MotorHAT.FORWARD)
    else:
        throttle.setSpeed(-cur_speed)
        throttle.run(Adafruit_MotorHAT.BACKWARD)

def set_steering(steering_deg):
    global cur_steer
    cur_steer = steering_deg
    if steering_deg < -10:
        left()
    elif steering_deg > 10:
        right()
    else:
        center()

# steering
def center():
    steering.setSpeed(0)
def left():
    steering.setSpeed(MAX_SPEED)
    steering.run(Adafruit_MotorHAT.BACKWARD)
def right():
    steering.setSpeed(MAX_SPEED)
    steering.run(Adafruit_MotorHAT.FORWARD)

# stop    
def stop():
    throttle.setSpeed(0)
    steering.setSpeed(0)
    throttle.run(Adafruit_MotorHAT.RELEASE)
    steering.run(Adafruit_MotorHAT.RELEASE)

