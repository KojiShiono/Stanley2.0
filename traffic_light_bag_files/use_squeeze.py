'''
Train SqueezeNet on augmented just_traffic_light
'''

import numpy as np
import os
import glob
import matplotlib.pyplot as plt
from sklearn.utils import shuffle as sk_shuffle
import cv2

import keras
from keras import backend as K
def set_keras_backend(backend):

    if K.backend() != backend:
        os.environ['KERAS_BACKEND'] = backend
        reload(K)
        assert K.backend() == backend

set_keras_backend("theano")

from keras.models import Sequential
from keras.layers import Lambda, Convolution2D,\
	MaxPooling2D, Dropout, Flatten, Dense, Conv2D,\
	Activation

from squeeze import SqueezeNet
from keras.optimizers import SGD



def preprocess_img(img, new_w, new_h):
    #img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)[:,:,0] # retain only hue channel
    return cv2.resize(img, (new_w, new_h))

# import X
print "Import and preprocess training images (just_traffic_light)"
imgs = glob.glob("just_traffic_light/*.png")

new_w = 224
new_h = 224

X = np.zeros((len(imgs), 3, new_h, new_w))
# X = np.zeros((10, 3, new_h, new_w))

for i in range(len(imgs)):
    img = plt.imread(imgs[i])
    img = preprocess_img(img, new_w, new_h)
    X[i,:,:,:] = img.T
    if not i%100:
        print i

# import Y
print "Import training labels"
Y_train = np.loadtxt("just_labels.txt")[:,1]
Y_train = keras.utils.to_categorical(Y_train, 4)

# data augmentation
# 64, 184, 134, 330
none_idx   = Y_train[:,0] == 1
red_idx    = Y_train[:,1] == 1
yellow_idx = Y_train[:,2] == 1

Y_none = Y_train[none_idx, :]
Y_red  = Y_train[red_idx, :]
Y_yellow = Y_train[yellow_idx, :]

none_imgs = X[none_idx,:,:,:]
red_imgs = X[red_idx,:,:,:]
yellow_imgs = X[yellow_idx,:,:,:]

X_train = np.vstack([X, none_imgs[:,:,:,::-1], none_imgs[:,:,::-1,:],
    none_imgs[:,:,::-1,::-1], red_imgs[:,:,:,::-1], yellow_imgs[:,:,:,::-1]])
Y_train = np.vstack([Y_train, Y_none, Y_none, Y_none, Y_red, Y_yellow])

# shuffle and split in test and training datasets
print "Shuffle training dataset"
X_train, Y_train = sk_shuffle(X_train, Y_train)

#=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-#

# Build the model (LeNet)
model = SqueezeNet(4)
model.summary() # print model summary

#=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-#

epochs = 10
batch_size = 64

sgd = SGD(lr=1e-3, decay=0.0002, momentum=0.9)
model.compile(optimizer=sgd, loss='categorical_crossentropy',
    metrics=['accuracy'])

model.fit(X_train, Y_train, batch_size=batch_size,
	nb_epoch=epochs, verbose=1, validation_split=0.05)

#=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-#

# Save model
print("Save model")
model_json = model.to_json()
import json
with open('squeeze.json', 'w') as f:
	json.dump(model_json, f, ensure_ascii=False)
model.save_weights("squeeze.h5")
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
