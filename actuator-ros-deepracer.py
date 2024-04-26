import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import ServoCtrlMsg


# throttle
MAX_SPEED = 100
cur_speed = MAX_SPEED

# ros2 publisher
publisher = None
servo_msg = None

# init
def init(default_speed=50):
    global publisher, servo_msg
    rclpy.init()
    node = rclpy.create_node('DeepPicar')
    publisher = node.create_publisher(ServoCtrlMsg, "ctrl_pkg/servo_msg", 1)
    servo_msg = ServoCtrlMsg()
    set_speed(default_speed)
    
def set_speed(speed):
    global cur_speed
    speed = int(MAX_SPEED * speed / 100)
    cur_speed = min(MAX_SPEED, speed)

def get_speed():
    return int(cur_speed * 100 / MAX_SPEED)

def stop():
    global servo_msg    
    servo_msg.throttle = 0.0
    servo_msg.angle = 0.0
    publisher.publish(servo_msg)
        
def ffw():
    global servo_msg    
    servo_msg.throttle = 0.5
    publisher.publish(servo_msg)

def rew():
    global servo_msg    
    servo_msg.throttle = -0.5
    publisher.publish(servo_msg)

# steering
def center():
    global servo_msg
    servo_msg.angle = 0.0
    publisher.publish(servo_msg)
def left():
    global servo_msg    
    servo_msg.angle = -0.5
    publisher.publish(servo_msg)
def right():
    global servo_msg
    servo_msg.angle = 0.5
    publisher.publish(servo_msg)

# exit    
def turn_off():
    stop()
    center()
