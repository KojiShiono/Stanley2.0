'''
Manual labeling of images from ROS bags.

labels:
no light = 0
red      = 1
yellow   = 2
green    = 3

loop_with_traffic_light/
     - 5833 none
5834 - 5938 red
5939 - 5952 green
5953 - 6208 none
6209 - 6248 red
6249 - 6294 green
6295 - 6563 none
6564 - 6641 green
6642 -      none

just_traffic_light/
     - 7613 green
7614 - 7674 yellow
7675 - 7723 red
7724 - 7787 none
7788 - 7798 red
7799 - 7923 green
7924 - 7984 yellow
7985 - 8108 red
8109 - 8234 green
8235 -      yellow

Returns
-------
*_labels.txt files with two columns. The first contains the
image name, the second the label.

Usage
-----
import numpy as np
labels = np.loadtxt(labels_file)
'''

import numpy as np
import os

imgs = os.listdir("loop_with_traffic_light/")
labels = np.zeros((len(imgs), 2), dtype=np.int64)

for i in range(len(imgs)):
    labels[i,0] = int(imgs[i].split('.')[0])

# labeling by hand...

# labels[:276, 1] = 0
labels[276:381, 1] = 1
labels[381:395, 1] = 3
# labels[395:651, 1] = 0
labels[651:691, 1] = 1
labels[691:737, 1] = 3
# labels[737:1006, 1] = 0
labels[1006:1084, 1] = 3
# labels[1084:, 1] = 0

np.savetxt("loop_labels.txt", labels, fmt='%d')

#-----------------------------------------------------------

imgs = os.listdir("just_traffic_light/")
labels = np.zeros((len(imgs), 2), dtype=np.int64)

for i in range(len(imgs)):
    labels[i,0] = int(imgs[i].split('.')[0])

labels[:79, 1] = 3
labels[79:140, 1] = 2
labels[140:189, 1] = 1
# labels[189:253, 1] = 0
labels[253:264, 1] = 1
labels[264:389, 1] = 3
labels[389:450, 1] = 2
labels[450:574, 1] = 1
labels[574:700, 1] = 3
labels[700:, 1] = 2

np.savetxt("just_labels.txt", labels, fmt='%d')
