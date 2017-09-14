'''
This is a simple utlity to extract camera data from ROS bags provided by
Udacity to trin the classifier on real images. It has been mostly taken from
http://ronny.rest/blog/post_2017_03_30_ros3_and_lidar/

Usage:
$ python bag_extractor.py bag_filename

Creates a folder named after the ROS bag file containing all the images in
.png format.
'''

import numpy as np
import cv2
import rosbag
import os, sys

def extract_from_bag(bag_filename):
    bag = rosbag.Bag(bag_filename, 'r')
    messages = bag.read_messages(topics=["/image_raw"])
    num_images = bag.get_message_count(topic_filters=["/image_raw"])

    if not os.path.isdir(bag_filename.split('.')[0]):
        os.mkdir(bag_filename.split('.')[0])

    for i in range(num_images):
        # READ NEXT MESSAGE IN BAG
        topic, msg, t  = messages.next()

        # CONVERT MESSAGE TO A NUMPY ARRAY
        img = np.fromstring(msg.data, dtype=np.uint8)
        img = img.reshape(msg.height, msg.width)

        # CONVERT TO RGB
        img = cv2.cvtColor(img, cv2.COLOR_BayerGR2RGB)

        cv2.imwrite("{0}/{1}.png".format(bag_filename.split('.')[0], msg.header.seq), img)

        print msg.header.seq

if __name__ == "__main__":
    extract_from_bag(sys.argv[1])
