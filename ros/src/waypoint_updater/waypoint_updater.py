#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 60 # Number of waypoints we will publish. You can change this number
LOOKBACK_WPS = -10

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)


        # rospy.Subscriber('/traffic_waypoint', Lane, self.traffic_waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.map = False
        self.pose_ = None
        self.waypoints = None
        self.lane = Lane()

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        if self.waypoints is None:
            pass
        else:
            # rospy.logwarn(self.waypoints.waypoints[0])
            self.pose = msg.pose
            # waypoints = self.waypoints.waypoints
            if not self.map:
                self.map_points = [point.pose.pose.position for point in self.waypoints.waypoints]
                self.n_map_points = len(self.map_points)
                self.dists = [self.distance(self.pose.position, point) for point in self.map_points]
                self.current_waypoint_index = np.argmin(np.array(self.dists))
                self.map = True
            else:
                last_dist = self.distance(self.pose.position, self.map_points[self.current_waypoint_index])
                min_dist_found = False
                while not min_dist_found:
                    self.current_waypoint_index += 1
                    dist = self.distance(self.pose.position, self.map_points[self.current_waypoint_index])
                    if dist < last_dist:
                        last_dist = dist
                    else:
                        self.current_waypoint_index -= 1
                        min_dist_found = True
            self.current_waypoint_index %= self.n_map_points
            loop = self.waypoints.waypoints + self.waypoints.waypoints[:LOOKAHEAD_WPS]
            self.lane.waypoints = loop[self.current_waypoint_index-LOOKBACK_WPS : self.current_waypoint_index + LOOKAHEAD_WPS]


            self.final_waypoints_pub.publish(self.lane)


    def waypoints_cb(self, Lane):
        # TODO: Implement
        self.waypoints = Lane
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, point1, point2):
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        return math.sqrt(dx*dx+dy*dy)

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
