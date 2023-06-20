import cv2
from threading import Thread,Lock
import time
import requests

use_thread = False
need_flip = False
cap = None
frame = None
URL = "http://192.168.1.200"

# public API
# init(), read_frame(), stop()

def init(res=(320, 240), fps=30, threading=True):
    print ("Initilize camera.")
    global cap, use_thread, frame, cam_thr

    cap = cv2.VideoCapture(URL + ":81/stream")
    requests.get(URL + "/control?var=framesize&val={}".format(1))

    # start the camera thread
    if threading:
        use_thread = True
        cam_thr = Thread(target=__update, args=())
        cam_thr.start()
        print ("start camera thread")
        time.sleep(1.0)
    else:
        print ("No camera threading.")
    if need_flip == True:
        print ("camera is Flipped")
    print ("camera init completed.")

def __update():
    global frame
    while use_thread:
        ret, tmp_frame = cap.read() # blocking read
        if need_flip == True:
            frame = cv2.flip(tmp_frame, -1)
        else:
            frame = tmp_frame
    print ("Camera thread finished...")
    cap.release()        

def read_frame():
    global frame
    if not use_thread:
       ret, frame = cap.read() # blocking read
    return frame

def stop():
    global use_thread
    print ("Close the camera.")
    use_thread = False

if __name__ == "__main__":
    init()
    while True:
        frame = read_frame()
        cv2.imshow('frame', frame)
        ch = cv2.waitKey(1) & 0xFF
        if ch == ord('q'):
            stop()
            break
    
