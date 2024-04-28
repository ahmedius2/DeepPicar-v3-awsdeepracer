# init

THROTTLE_PWM_FILE = "/sys/class/pwm/pwmchip0/pwm0/duty_cycle"
STEERING_PWM_FILE = "/sys/class/pwm/pwmchip0/pwm1/duty_cycle"

# throttle range:
# 1046000               max forward
# 1446000               stop
# 1846000               max reverse

# steering range:
# 1046000               right max
# 1446000               center
# 1846000               left max

def init(default_speed=50):
    return

def set_throttle(throttle_pct):
    # throttle_pct: [-100, +100]
    # throttle_pwm: [1046000, 1846000]
    throttle_pwm = int(1446000 - (throttle_pct * 400000 / 100))
    print(f"throttle_pct: {throttle_pct} throttle_pwm: {throttle_pwm}")

    # Ensure throttle_pwm is within the acceptable range
    throttle_pwm = max(1046000, min(1846000, throttle_pwm))
    try:
        with open(THROTTLE_PWM_FILE, 'w') as f:
            f.write(str(throttle_pwm))
    except OSError as e:
        print(f"Failed to set throttle: {e}")

def set_steering(steering_deg):
    # steering_deg: [-30, +30]
    # steering_pwm: [1046000, 1846000]
    steering_pwm = int(1446000 - (steering_deg * 400000 / 30))
    print(f"steering_deg: {steering_deg} steering_pwm: {steering_pwm}")

    # Ensure steering_pwm is within the acceptable range
    steering_pwm = max(1046000, min(1846000, steering_pwm))
    try:
        with open(STEERING_PWM_FILE, 'w') as f:
            f.write(str(steering_pwm))
    except OSError as e:
        print(f"Failed to set steering: {e}")

def stop():
    with open(THROTTLE_PWM_FILE, 'w') as f:
        f.write("1446000")
    with open(STEERING_PWM_FILE, 'w') as f:
        f.write("1446000")
