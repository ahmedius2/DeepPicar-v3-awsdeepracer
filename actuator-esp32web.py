import requests
import time

URL = "http://192.168.1.200"
cur_speed = 100 # throttle

def set_speed(speed):
    global cur_speed
    if speed >= 0 and speed <= 100 :
        requests.get(URL + "/control?var=throttle&val={}".format(speed))
        cur_speed = speed

def get_speed():
    global cur_speed
    return cur_speed

# init
def init(default_speed=100):
    set_speed(default_speed)
    stop()
def stop():
    requests.get(URL + "/control?var=stop&val={}".format(0))
    print("stop")
def ffw():
    requests.get(URL + "/control?var=forward&val={}".format(0))
    print("ffw")
def rew():
    requests.get(URL + "/control?var=backward&val={}".format(0))
    print("rew")
# steering
def center():
    requests.get(URL + "/control?var=center&val={}".format(0))
    print("center")
def left():
    requests.get(URL + "/control?var=left&val={}".format(0))
    print("left")
def right():
    requests.get(URL + "/control?var=right&val={}".format(0))
    print("right")

# exit    
def turn_off():
    stop()
    center()

if __name__ == "__main__":
    init()
    time.sleep(1)
    left()
    time.sleep(1)    
    right()
    time.sleep(1)    
    center()
    time.sleep(1)
    ffw()
    time.sleep(1)    
    rew()
    time.sleep(1)    

