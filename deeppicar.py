#!/usr/bin/python
import os
import time
import atexit
import cv2
import math
import numpy as np
import sys
import params
import argparse
import input_stream

from PIL import Image, ImageDraw

##########################################################
# import deeppicar's sensor/actuator modules
##########################################################
camera   = __import__(params.camera)
actuator = __import__(params.actuator)
#inputdev = __import__(params.inputdev)

##########################################################
# global variable initialization
##########################################################
use_thread = True
view_video = False
fpv_video = False
enable_record = False
cfg_cam_res = (160, 120)
cfg_cam_fps = 30

frame_id = 0
max_frames = 2000
angle = 0.0
period = 0.05 # sec (=50ms)

##########################################################
# local functions
##########################################################
def deg2rad(deg):
    return deg * math.pi / 180.0
def rad2deg(rad):
    return 180.0 * rad / math.pi

def g_tick():
    t = time.time()
    count = 0
    while True:
        count += 1
        yield max(t + count*period - time.time(),0)

def turn_off():
    actuator.disable()
    camera.stop()
    if frame_id > 0:
        keyfile.close()
        vidfile.release()

def crop_image(img):
    scaled_img = cv2.resize(img, (max(int(params.img_height * 4 / 3), params.img_width), params.img_height))
    fb_h, fb_w, fb_c = scaled_img.shape
    # print(scaled_img.shape)
    startx = int((fb_w - params.img_width) / 2);
    starty = int((fb_h - params.img_height) / 2);
    return scaled_img[starty:starty+params.img_height, startx:startx+params.img_width,:]

def preprocess(img):
    if args.pre == "crop":
        img = crop_image(img)
    else:
        img = img[img.shape[0]//6:]
        img = cv2.resize(img, (params.img_width, params.img_height))
    # Convert to grayscale and readd channel dimension
    if params.img_channels == 1:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = np.reshape(img, (params.img_height, params.img_width, params.img_channels))
    img = img / 255.
    return img

def overlay_image(l_img, s_img, x_offset, y_offset):
    assert y_offset + s_img.shape[0] <= l_img.shape[0]
    assert x_offset + s_img.shape[1] <= l_img.shape[1]

    l_img = l_img.copy()
    for c in range(0, 3):
        l_img[y_offset:y_offset+s_img.shape[0],
              x_offset:x_offset+s_img.shape[1], c] = (
                  s_img[:,:,c] * (s_img[:,:,3]/255.0) +
                  l_img[y_offset:y_offset+s_img.shape[0],
                        x_offset:x_offset+s_img.shape[1], c] *
                  (1.0 - s_img[:,:,3]/255.0))
    return l_img

def print_stats(execution_times):
    # Calculate statistics
    avg = np.mean(execution_times)
    min = np.min(execution_times)
    max = np.max(execution_times)
    p99 = np.percentile(execution_times, 99)
    p90 = np.percentile(execution_times, 90)
    p50 = np.percentile(execution_times, 50)

    print(f"Average Execution Time: {avg:.6f} seconds")
    print(f"Minimum Execution Time: {min:.6f} seconds")
    print(f"Maximum Execution Time: {max:.6f} seconds")
    print(f"99th Percentile Execution Time: {p99:.6f} seconds")
    print(f"90th Percentile Execution Time: {p90:.6f} seconds")
    print(f"50th Percentile Execution Time: {p50:.6f} seconds")

def measure_execution_time(func, num_trials):
    execution_times = []
    for _ in range(num_trials):
        start_time = time.time()
        func()  # Call the function to measure its execution time
        end_time = time.time()
        execution_time = end_time - start_time
        execution_times.append(execution_time)
    # Calculate statistics
    print_stats(execution_times)

##########################################################
# program begins
##########################################################

parser = argparse.ArgumentParser(description='DeepPicar main')
parser.add_argument("-d", "--dnn", help="Enable DNN", action="store_true")
parser.add_argument("-t", "--throttle", help="throttle percent. [0-100]%", type=int, default=100)
parser.add_argument("--turnthresh", help="throttle percent. [0-30]degree", type=int, default=10)
parser.add_argument("-n", "--ncpu", help="number of cores to use.", type=int, default=2)
parser.add_argument("-f", "--hz", help="control frequnecy", type=int)
parser.add_argument("--fpvvideo", help="Take FPV video of DNN driving", action="store_true")
parser.add_argument("--use", help="use [tflite|tf|openvino]", type=str, default="tflite")
parser.add_argument("--pre", help="preprocessing [resize|crop]", type=str, default="resize")
parser.add_argument("-g", "--gamepad", help="Use gamepad", action="store_true")
args = parser.parse_args()

if args.throttle:
    print ("throttle = %d pct" % (args.throttle))
if args.turnthresh:
    args.turnthresh = args.turnthresh
    print ("turn angle threshold = %d degree\n" % (args.turnthresh))
if args.hz:
    period = 1.0/args.hz
    print("new period: ", period)
if args.fpvvideo:
    fpv_video = True
    print("FPV video of DNN driving is on")

print("period (sec):", period)
print("preprocessing:", args.pre)

if args.gamepad:
    cur_inp_type= input_stream.input_type.GAMEPAD
else:
    cur_inp_type= input_stream.input_type.KEYBOARD

cur_inp_stream= input_stream.instantiate_inp_stream(cur_inp_type, args.throttle)

##########################################################
# import deeppicar's DNN model
##########################################################
print ("Loading model: " + params.model_file)

print("use:", args.use)
if args.use == "tf":
    from tensorflow import keras
    model = keras.models.load_model(params.model_file+'.h5')
elif args.use == "openvino":
    import openvino as ov
    core = ov.Core()
    ov_model = core.read_model(params.model_file+'.tflite')
    model = core.compile_model(ov_model, 'AUTO')
else:
    try:
        # Import TFLite interpreter from tflite_runtime package if it's available.
        from tflite_runtime.interpreter import Interpreter
        interpreter = Interpreter(params.model_file+'.tflite', num_threads=args.ncpu)
    except ImportError:
        # Import TFLMicro interpreter
        try:
            from tflite_micro_runtime.interpreter import Interpreter 
            interpreter = Interpreter(params.model_file+'.tflite')
        except:
            # If all failed, fallback to use the TFLite interpreter from the full TF package.
            import tensorflow as tf
            interpreter = tf.lite.Interpreter(model_path=params.model_file+'.tflite', num_threads=args.ncpu)

    interpreter.allocate_tensors()
    input_index = interpreter.get_input_details()[0]["index"]
    output_index = interpreter.get_output_details()[0]["index"]


# initlaize deeppicar modules
actuator.init(args.throttle)
camera.init(res=cfg_cam_res, fps=cfg_cam_fps, threading=use_thread)
atexit.register(turn_off)

g = g_tick()
start_ts = time.time()

frame_arr = []
angle_arr = []
actuator_times = []

# enter main loop
while True:
    if use_thread:
        time.sleep(next(g))
    frame = camera.read_frame()
    if frame is None:
        print("frame is None")
        break
    ts = time.time()

    # receive input (must be non blocking)
    #ch = inputdev.read_single_event()
    if view_video:
        cv2.imshow('frame', frame)
        ch = cv2.waitKey(1) & 0xFF
    else:
        command, direction, speed = cur_inp_stream.read_inp()

    if command == 'a':
        actuator.ffw()
        start_ts = ts
        print ("accel")
    elif command == 's':
        actuator.disable()
        print ("stop")
        print ("duration: %.2f" % (ts - start_ts))
        enable_record = False # stop recording as well 
        args.dnn = False # manual mode
    elif command == 'z':
        print ("TODO reverse")
    elif command == 'r':
        enable_record = not enable_record
        print ("record mode: ", enable_record)
    elif command == 't':
        print ("toggle video mode")
        view_video = not view_video
    elif command == 'd':
        print ("toggle DNN mode")
        args.dnn = not args.dnn
    elif command == 'q':
        actuator.disable()
        break

    if args.dnn == True:
        # 1. machine input
        img = preprocess(frame)
        img = np.expand_dims(img, axis=0).astype(np.float32)
        if args.use == "tf":
            angle = model.predict(img)[0]
        elif args.use == "openvino":
            angle = model(img)[0][0][0]
            # print ('angle:', angle);
        else: # tflite
            interpreter.set_tensor(input_index, img)
            interpreter.invoke()
            angle = interpreter.get_tensor(output_index)[0][0]

        steering_deg = rad2deg(angle)
        actuator.set_steering(steering_deg)
    else:
        actuator.set_steering(direction * 30)
        angle = deg2rad(direction * 30)
    actuator.set_throttle(speed)

    dur = time.time() - ts
    if dur > period:
        print("%.3f: took %d ms - deadline miss."
                % (ts - start_ts, int(dur * 1000)))
    # else:
    #     print("%.3f: took %d ms" % (ts - start_ts, int(dur * 1000)))
    
    if enable_record == True and frame_id == 0:
        # create files for data recording
        keyfile = open(params.rec_csv_file, 'w+')
        keyfile.write("ts,frame,wheel\n") # ts (ms)
        try:
            fourcc = cv2.cv.CV_FOURCC(*'XVID')
        except AttributeError as e:
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
        vidfile = cv2.VideoWriter(params.rec_vid_file, fourcc,
                                cfg_cam_fps, cfg_cam_res)
    if enable_record == True and frame is not None:
        # increase frame_id
        frame_id += 1

        # write input (angle)
        str = "{},{},{}\n".format(int(ts*1000), frame_id, angle)
        keyfile.write(str)

        if args.dnn and fpv_video:
            textColor = (255,255,255)
            bgColor = (0,0,0)
            newImage = Image.new('RGBA', (100, 20), bgColor)
            drawer = ImageDraw.Draw(newImage)
            drawer.text((0, 0), "Frame #{}".format(frame_id), fill=textColor)
            drawer.text((0, 10), "Angle:{}".format(angle), fill=textColor)
            newImage = cv2.cvtColor(np.array(newImage), cv2.COLOR_BGR2RGBA)
            frame = overlay_image(frame,
                                     newImage,
                                     x_offset = 0, y_offset = 0)
        # write video stream
        vidfile.write(frame)
        #img_name = "cal_images/opencv_frame_{}.png".format(frame_id)
        #cv2.imwrite(img_name, frame)
        if frame_id >= max_frames:
            print ("recorded 2000 frames")
            break
        print ("%.3f %d %.3f %d(ms)" %
           (ts, frame_id, angle, int((time.time() - ts)*1000)))
    # update previous steering angle


print ("Finish..")
if len(actuator_times):
    print ("Actuator latency measurements: {} trials".format(len(actuator_times)))
    print_stats(actuator_times)
turn_off()
