#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
from traffic_light_config import config
import time
import math
import numpy as np
import darknet
net = darknet.load_net("cfg/yolo.cfg", "yolo.weights", 0)
meta = darknet.load_meta("cfg/coco.data")

from PIL import Image as PILImage

STATE_COUNT_THRESHOLD = 1

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights helps you acquire an accurate ground truth data source for the traffic light
        classifier, providing the location and current color state of all traffic lights in the
        simulator. This state can be used to generate classified images or subbed into your solution to
        help you work on another single component of the node. This topic won't be available when
        testing your solution in real life so don't rely on it in the final submission.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/camera/image_raw', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light to /traffic_waypoint
        Args:
            msg (Image): image from car-mounted camera
        """
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(-1))
        self.state_count += 1

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        #TODO implement
        p = (pose.position.x, pose.position.y, pose.position.z)

        min_dist = 1e5
        closest_wpnt = -1

        for i in range(len(self.waypoints.waypoints)):
            w = self.waypoints.waypoints[i].pose.pose.position
            q = (w.x, w.y, w.z)
            if self.distance(p, q) < min_dist:
                min_dist = self.distance(p, q)
                closest_wpnt = i

        return min_dist
        # return 0

    def distance(self, p, q):
        return np.sqrt((p[0]-q[0])**2+(p[1]-q[1])**2+(p[2]-q[2])**2)




    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location
        Args:
            point_in_world (Point): 3D location of a point in the world
        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image
        """

        fx = config.camera_info.focal_length_x
        fy = config.camera_info.focal_length_y

        image_width = config.camera_info.image_width
        image_height = config.camera_info.image_height

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        self.camera_image.encoding = "rgb8"
        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light = None
        light_positions = config.light_positions
        if self.pose is not None and self.waypoints is not None:
            light_wp = self.get_closest_waypoint(self.pose.pose)


        #TODO find the closest visible traffic light (if one exists)
        dist = -1
        img = self.bridge.imgmsg_to_cv2(self.camera_image, "passthrough")
        # cv2_img = cv2.cvtColor(cv2_img, cv2.COLOR_BGR2RGB)

        cv2.imwrite("./image.png", img)

        detected = darknet.detect(net, meta, "./image.png")
        lights = ["red", "green"]
        if detected != []:
            for d in detected:
                if d[0] == "traffic light":
                    x = d[2][0]
                    y = d[2][1]
                    w = d[2][2]
                    h = d[2][3]
                    im = PILImage.open("./image.png")
                    if im.width >= x+h/2:
                        im = im.crop((x-h/2, y-h/2, x+h/2, y+h/2))
                    else:
                        im = im.crop((im.width-h, y-h/2, im.width, y+h/2))
                    #im.save("./light.png")
                    tl = np.fromstring(im.tobytes(), dtype=np.uint8)
                    tl = tl.reshape((im.height, im.width, 3))
                    tl = cv2.cvtColor(tl, cv2.COLOR_BGR2GRAY)
                    #cv2.imwrite("./light.png", tl)
                    third = np.int(im.height/3.)
                    tl_red = tl[:third,third:-third]
                    #tl_yel = tl[third:-third,third:-third]
                    tl_gre = tl[-third:,third:-third]
                    light = np.array([np.sum(tl_red), np.sum(tl_gre)])
                    print(lights[np.argmax(light)])
                    break
        '''
        mask = self.red_select(img)
        red_pixels = np.sum(mask)
        if red_pixels > 10:
            red_pos = np.where(mask==1)
            red_x = np.mean(red_pos[0])

            if red_x < 350 and red_x > 200:
                return int(35 - (red_x-200)/40), TrafficLight.RED
            else:
                return 45, TrafficLight.RED

        else:
            return -1, TrafficLight.UNKNOWN
        '''
        return -1, TrafficLight.UNKNOWN

        # dists = [self.sign_distance(light.pose.pose.position, self.pose.pose.position) for light in self.lights]
        # dist = np.min(dists)
        # if dist >20 and dist < 40:
        #     cv2.imwrite("/home/student/Desktop/Stanley2.0/images/"+str(int(dist))+"_"+str(time.time())+".jpg", cv2_img)
        # else:
        #     if int(dist)%66 == 0:
        #         cv2.imwrite("/home/student/Desktop/Stanley2.0/images/"+str(int(dist))+"_"+str(time.time())+".jpg", cv2_img)
        # cv2.imwrite("/home/student/Desktop/Stanley2.0/images/"+str(time.time())+".jpg", cv2_img)


    def sign_distance(self, point1, point2):
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        d = math.sqrt(dx*dx+dy*dy)
        return d if dx > 0 else 99999.

    def red_select(self, img):
        r_channel = img[:,:,0]
        g_channel = img[:,:,1]
        binary_output = np.zeros_like(r_channel)
        binary_output[(r_channel > 200) & (g_channel <= 50)] = 1
        return binary_output

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
