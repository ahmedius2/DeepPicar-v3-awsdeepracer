#!/usr/bin/env python
##########################################################
# camera module selection
#   "camera-webcam" "camera-null"
##########################################################
camera="camera-webcam"
# URL="http://192.168.4.1"

##########################################################
# actuator selection
#   "actuator-drv8835", "actuator-adafruit_hat"
#   "actuator-null" "actuator-servo-deepracer"
##########################################################
actuator="actuator-servo-deepracer"

##########################################################
# intputdev selection
#   "input-kbd", "input-joystick", "input-web"
##########################################################
inputdev="input-kbd"

##########################################################
# input config 
##########################################################
img_width = 200
img_height = 66
img_channels = 3

##########################################################
# model selection
#   "model_large"   <-- nvidia dave-2 model
##########################################################
model_name = "model_large"
model_file = "models/{}-{}x{}x{}".format(model_name[6:], img_width, img_height, img_channels)

##########################################################
# recording config 
##########################################################
rec_vid_file="dataset/out-video.avi"
rec_csv_file="dataset/out-key.csv"
