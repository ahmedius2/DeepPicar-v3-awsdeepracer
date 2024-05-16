# init
import os
import json

GPIO_DIR = "/sys/class/gpio"
PWM_DIR = "/sys/class/pwm/pwmchip0"
PWM_PERIOD = "20000000"
#PWM_CALIB_FILE = "/opt/aws/deepracer/calibration.json"
PWM_CALIB_FILE = "pwm_calibration.json"

# The default values below will be overwritten
# with the values in calibration file
thr_lims = [1360500, 1468500, 1468500, 1536000]
srv_lims = [1230000, 1390000, 1390000, 1670000]
polarities = [-1, 1]
calib_mode=False

def write_to_file(path, val):
    #print(path, val)
    with open(path, 'w') as f:
        f.write(val)

def init(default_speed=0):
    global calib_mode
    # read the calibration file
    if calib_mode==False:
        try:
            with open(PWM_CALIB_FILE, 'r') as f:
                calib = json.load(f)

            global thr_lims
            global srv_lims
            global polarities
            thr_lims = calib['throttle_limits']
            polarities[0] = calib['polarity'][0]

            #TODO servo calibration needs to be done!

        except:
            print('Could not read calibration file, using default values.')

        print('Throttle pwm limits:', thr_lims)
        print('Steering pwm limits:', srv_lims)
        print('Polarities:', polarities)

    if not os.path.exists(GPIO_DIR + "/gpio436"):
        try:
            write_to_file(GPIO_DIR + "/export", "436")
        except OSError as e:
            print(f"Failed to export gpio436")
    gpio436_path = GPIO_DIR + "/gpio436"
    try:
        write_to_file(gpio436_path + "/direction", "out")
        write_to_file(gpio436_path + "/value", "0")
    except OSError as e:
        print(f"Failed to configure gpio436")

    # throttle and steering
    for pwm in ("/pwm0", "/pwm1"):
        if not os.path.exists(PWM_DIR + pwm):
            try:
                write_to_file(PWM_DIR + "/export", pwm[-1])
            except OSError as e:
                print(f"Failed to export {pwm}")

        try:
            write_to_file(PWM_DIR + pwm + "/period", PWM_PERIOD)
            # Disable it first to set the polarity
            # The direction of steering is determined by
            # the pwm value, not the polarity.
            write_to_file(PWM_DIR + pwm + "/enable", "0")
            write_to_file(PWM_DIR + pwm + "/polarity", "normal")
            write_to_file(PWM_DIR + pwm + "/enable", "1")
            write_to_file(PWM_DIR + pwm + "/duty_cycle", \
                    str(thr_lims[1]))
        except OSError as e:
            print(f"Failed to configure {pwm}")

    return

def get_pwm(pct, limits, polarities):
    if polarities[0] == 1:
        if pct > 0:
            limdiff = limits[3]-limits[2]
            pwm = thr_lims[2] + (pct * limdiff / 100)
        elif pct < 0:
            limdiff = limits[1]-limits[0]
            pwm = thr_lims[1] + (pct * limdiff / 100)
        else:
            pwm = thr_lims[1] + (thr_lims[2]-thr_lims[1]) / 2
    else:
        if pct > 0:
            limdiff = limits[0]-limits[1]
            pwm = thr_lims[1] + (pct * limdiff / 100)
        elif pct < 0:
            limdiff = limits[2]-limits[3]
            pwm = thr_lims[2] + (pct * limdiff / 100)
        else:
            pwm = thr_lims[1] + (thr_lims[2]-thr_lims[1]) / 2
    return int(pwm)

# throttle_pct: [-100, +100]
def set_throttle(throttle_pct):
    global thr_lims
    global polarities
    # Ensure throttle_pwm is within the acceptable range
    p = max(-100, min(100, throttle_pct))
    pwm = get_pwm(p, thr_lims, polarities)
    print(f"throttle_pct: {throttle_pct} throttle_pwm: {pwm}")

    try:
        write_to_file(PWM_DIR + '/pwm0/duty_cycle', str(pwm))
    except OSError as e:
        print(f"Failed to set throttle: {e}")

# steering_deg: [-30, +30]
def set_steering(steering_deg):
    global srv_lims
    global polarities
    # Ensure input is within the acceptable range
    p = max(-30, min(30, steering_deg))
    p = p / 30 * 100
    pwm = get_pwm(p, srv_lims, polarities)
    print(f"steering_deg: {steering_deg} steering_pwm: {pwm}")

    try:
        write_to_file(PWM_DIR + '/pwm1/duty_cycle', str(pwm))
    except OSError as e:
        print(f"Failed to set steering: {e}")

def ffw():
    write_to_file(PWM_DIR + '/pwm0/enable', "1")
    write_to_file(PWM_DIR + '/pwm1/enable', "1")

def stop():
    global thr_lims
    write_to_file(PWM_DIR + '/pwm0/duty_cycle', str(thr_lims[1]))
    write_to_file(PWM_DIR + '/pwm1/duty_cycle', str(srv_lims[1]))

def disable():
    stop()
    write_to_file(PWM_DIR + '/pwm0/enable', "0")
    write_to_file(PWM_DIR + '/pwm1/enable', "0")

def drive_calib(init_pwm):
    import params
    inputdev = __import__(params.inputdev)
    pwm = init_pwm
    while True:
        ch = inputdev.read_single_event()
        if ch == ord('a'): # increase
            pwm += 2000
        elif ch == ord('z'): # decrease
            pwm -= 2000
        elif ch == ord('q'):
            break

        try:
            write_to_file(PWM_DIR + '/pwm0/duty_cycle', str(pwm))
        except OSError as e:
            print(f"Failed to set throttle: {e}")

    return pwm


if __name__ == "__main__":
    print('Calibrating now.')
    calib_mode = True
    init()
    print('\n\nSet the max forward')
    pwm1 = drive_calib(thr_lims[1])
    print('\n\nSet the min forward. Decrease the speed from max' \
            ' gradually until it stops. Then increase it until' \
            'it starts moving again.')
    pwm2 = drive_calib(pwm1)
    print('\n\nSet the max backward')
    pwm3 = drive_calib(pwm2)
    print('\n\nSet the min forward. Decrease the speed from max' \
            ' gradually until it stops. Then increase it until' \
            ' it starts moving again.')
    pwm4 = drive_calib(pwm3)

    polarities[0] = -1 if pwm1 < pwm3 else 1
    if polarities[0] == -1:
        thr_lims = [pwm1, pwm2, pwm4, pwm3]
    else:
        thr_lims = [pwm3, pwm4, pwm2, pwm1]
    disable()

    print(thr_lims)
    print(polarities)

    with open(PWM_CALIB_FILE, 'w') as f:
        calib = {}
        calib['throttle_limits'] = thr_lims
        calib['polarity'] = polarities
        json.dump(calib, f)
