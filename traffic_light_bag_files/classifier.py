'''
Train CNN on just_traffic_light dataset
'''

import numpy as np
import os
import glob
import matplotlib.pyplot as plt
from sklearn.utils import shuffle as sk_shuffle
import cv2

import tensorflow as tf
import keras
from keras.models import Sequential
from keras.layers import Lambda, Convolution2D,\
	MaxPooling2D, Dropout, Flatten, Dense, Conv2D,\
	Activation

def preprocess_img(img, new_w, new_h):
    img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)[:,:,0] # retain only hue channel
    return cv2.resize(img, (new_w, new_h))

# import X
print "Import and preprocess training images (just_traffic_light)"
imgs = glob.glob("just_traffic_light/*.png")

new_w = 137
new_h = 110

X = np.zeros((len(imgs), new_h, new_w))

for i in range(len(imgs)):
	img = plt.imread(imgs[i])
	img = preprocess_img(img, new_w, new_h)
	X[i,:,:] = img
	if not i%10:
		print i

# import Y
print "Import training labels"
Y = np.loadtxt("just_labels.txt")[:,1]

# shuffle and split in test and training datasets
print "Shuffle training dataset"
X, Y = sk_shuffle(X, Y)

X_train = X[:,:,:, None]
Y_train = Y

Y_train = keras.utils.to_categorical(Y_train, 4)

#=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-#

# Build the model
model = Sequential()
model.add(Lambda(lambda x: x/127.5 - 1.,
	input_shape=(new_h, new_w, 1), name="Normalization"))
model.add(Conv2D(20, (5, 5), padding="same", activation="relu"))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

# second set of CONV => RELU => POOL
model.add(Conv2D(50, (5, 5), padding="same", activation="relu"))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(2, 2)))

# set of FC => RELU layers
model.add(Flatten())
# model.add(Dense(4))
# model.add(Activation("relu"))

# softmax classifier
model.add(Dense(4))
model.add(Activation("softmax"))

model.summary() # print model summary

#=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-#

epochs = 10
batch_size = 64

# Training
model.compile(loss='mean_squared_error',optimizer='adam')

# train the model using the 10% of the dataset for validation
model.fit(X_train, Y_train, batch_size=batch_size,
	nb_epoch=epochs, verbose=1, validation_split=0.05)

#=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-#

# Save model
print("Save model")
model_json = model.to_json()
import json
with open('model.json', 'w') as f:
	json.dump(model_json, f, ensure_ascii=False)
model.save_weights("model.h5")
#
# #=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-#
#
# # Test run
# P = model.predict(X_test[:,:,:,0][:,:,:,None])
# P_labels = np.argmax(P, 1)
# Y_labels = np.argmax(Y_test, 1)
#
# ## plot predictions over line of equality
# # sns.set_style("white")
# fig = plt.figure(1, figsize=(5,5))
# fig.clf()
# ax = fig.add_subplot(111)
# plt.plot([-3.5,3.5],[-3.5,3.5], 'k--', label="line of equality")
# ax.scatter(P_labels, Y_labels, marker='o', color="orchid", s=70, zorder=10)
# plt.xlabel("prediction")
# plt.ylabel("y_test")
# plt.legend(loc='best')
# plt.tight_layout()
# plt.draw()
# plt.show()
