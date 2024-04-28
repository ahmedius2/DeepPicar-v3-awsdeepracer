import rclpy
from rclpy.node import Node
from deepracer_interfaces_pkg.msg import ServoCtrlMsg


# throttle
cur_speed = 0
cur_steer = 0

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
    set_throttle(default_speed)
    
def set_throttle(throttle_pct):
    global cur_speed, servo_msg
    cur_speed = throttle_pct
    servo_msg.throttle = cur_speed/100
    publisher.publish(servo_msg)

def set_steering(steering_deg):
    global cur_steer, servo_msg
    cur_steer = steering_deg
    servo_msg.angle = deg2rad(steering_deg)
    publisher.publish(servo_msg)

# exit    
def turn_off():
    set_throttle(0)
    set_steering(0)
