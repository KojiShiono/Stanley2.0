import cv2
import numpy as np
import matplotlib.pyplot as plt

imgR = cv2.imread("simulator_images/images/00000001.jpeg")
imgG = cv2.imread("simulator_images/images/00000218.jpeg")
imgY = cv2.imread("simulator_images/images/00000223.jpeg")

red_threshold = 240

image = imgR
color_thresholds = (image[:,:,0] > red_threshold)

plt.imshow(color_thresholds)

plt.show()
