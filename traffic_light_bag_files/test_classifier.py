'''
Test trained net on loop_with_traffic_light dataset
'''

import numpy as np
import os
import glob
import matplotlib.pyplot as plt
import cv2
import json
import tensorflow as tf
from keras.models import model_from_json

def preprocess_img(img, new_w, new_h):
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)[:,:,0] # retain only hue channel
    return cv2.resize(img, (new_w, new_h))

# load CNN
with open("model.json", 'r') as jfile:
    model = model_from_json(json.loads(jfile.read()))
model.compile("adam", "mse")
weights_file = "model.h5"
model.load_weights(weights_file)

# read test images
imgs = glob.glob("loop_with_traffic_light/*.png")

new_w = 137
new_h = 110

font = cv2.FONT_HERSHEY_SIMPLEX
labels = ["None", "red", "yellow", "green"]
for i in range(len(imgs)):
    img = cv2.imread(imgs[i])
    img_p = preprocess_img(img, new_w, new_h)

    P = model.predict(img_p[None,:,:,None])
    idx = np.argmax(P)
    cv2.putText(img, labels[idx],(100,100), font, 2,(255,255,255), 4)
    cv2.imwrite("test_images/{:04d}.png".format(i+1), img)

    if not i%100:
        print i
