import serial

cur_speed = 0
cur_steer = 0

# init
def init(default_speed=50):
    global ser
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    stop()
    set_throttle(default_speed)
    print ("actuator-arduino init completed.")

def set_throttle(throttle_pct):
    global cur_speed
    ser.write("t " + str(throttle_pct))
    cur_speed = throttle_pct

def set_steering(steering_deg):
    global cur_steer
    ser.write("s " + str(steering_deg))
    cur_steer = steering_deg

# exit    
def turn_off():
    set_throttle(0)
    set_steering(0)
    ser.close()
