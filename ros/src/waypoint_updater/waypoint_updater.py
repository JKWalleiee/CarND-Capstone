#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32


import math

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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
# Selected ahead waypoints' indexes to publish (in order to save interpolations' computating time)
LOOKAHEAD_WPS_MASK = [0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 12, 16, 20, 28, 36, 52, 68, 100, 132, 196]
MAX_DECEL = 0.5 # Max deceleration
STOPPING_WPS_BEFORE = 4 # Number of waypoints to stop before a traffic light line
#TARGET_SPEED_MPH = 10

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_tree = None
        self.stop_line_wp_idx = -1

        #rospy.spin()
        self.loop()

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoints_tree:
                # Getting the final waypoints
                final_lane = self.generate_lane()
                self.publish_waypoints(final_lane)
            rate.sleep()
            
    ### Callback Suscriber ###

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        self.waypoints_tree = KDTree(
            [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y]
            for waypoint in waypoints.waypoints])

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_line_wp_idx = msg.data

    #def obstacle_cb(self, msg):
    #    # TODO: Callback for /obstacle_waypoint message. We will implement it later
    #    pass

    ### --- ###

    ### Helper Functions ###

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoints_tree.query([x, y], 1)[1]

        # Checking if closest point is ahead or behind the vehicle
        closest_waypoint = self.base_waypoints.waypoints[closest_idx]
        prev_waypoint = self.base_waypoints.waypoints[
            (closest_idx - 1) if closest_idx > 0 else (len(self.base_waypoints.waypoints) - 1)]
        closest_coord = [closest_waypoint.pose.pose.position.x, closest_waypoint.pose.pose.position.y]
        prev_coord = [prev_waypoint.pose.pose.position.x, prev_waypoint.pose.pose.position.y]

        # Equation for hyperplane through closest coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_coord)
        pos_vect = np.array([x, y])

        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.base_waypoints.waypoints)
        return closest_idx
    
    def generate_lane(self):
        closest_idx = self.get_closest_waypoint_idx()
        # We want the car to stop at the end of the track, so not doing module
        farthest_idx = min(closest_idx + LOOKAHEAD_WPS, len(self.base_waypoints.waypoints))

        if self.stop_line_wp_idx == -1 or self.stop_line_wp_idx >= farthest_idx or self.stop_line_wp_idx < closest_idx + 1:
            # If there is no red traffic light ahead, just adding next selected waypoints
            return self.accelerate_to_target_velocity(closest_idx, farthest_idx)

        else:
            # If there is a red traffic light ahead, modifying the waypoints velocity to gradually stop
            return self.decelerate_to_stop(closest_idx, farthest_idx)

    def accelerate_to_target_velocity(self, closest_idx, farthest_idx):
        final_waypoints = []

        for i in LOOKAHEAD_WPS_MASK:
            idx = closest_idx + i
            if idx < farthest_idx:
                wp = self.base_waypoints.waypoints[idx]
                final_waypoints.append(wp)
        return final_waypoints
    
    #def accelerate_to_target_velocity(self, closest_idx, farthest_idx):
    #    # set the velocity for lookahead waypoints
    #    lookahead_waypoints = self.waypoints[closest_idx:closest_idx+LOOKAHEAD_WPS]
    #    for i in range(len(lookahead_waypoints) - 1):                
    #        # convert 10 miles per hour to meters per sec
    #        self.set_waypoint_velocity(lookahead_waypoints, i, (TARGET_SPEED_MPH * 1609.34) / (60 * 60))
    
    def decelerate_to_stop(self, closest_idx, farthest_idx):
        final_waypoints = []
        # Index of the closest waypoint point before the stop line of the traffic light
        stop_idx = max(self.stop_line_wp_idx - STOPPING_WPS_BEFORE, closest_idx)
        target_wp = self.base_waypoints.waypoints[stop_idx]
        dist = 0.0

        for i in LOOKAHEAD_WPS_MASK[::-1]:
            # For each one of the selected waypoints (starting from the farthest one),
            # calculating the distance to the stop line and adjust the velocity in order to gradually stop
            idx = closest_idx + i
            if idx < farthest_idx:
                wp = self.base_waypoints.waypoints[idx]
                p = Waypoint()
                p.pose = wp.pose
                vel = 0.0

                if idx < stop_idx:
                    # Calculating the distance from the stop line to the current waypoint
                    dist += self.distance(target_wp.pose.pose.position, wp.pose.pose.position)
                    
                    # Reducing the velocity according to the max acceleration
                    vel = math.sqrt(2 * MAX_DECEL * dist)
                    if vel < 1.0:
                        vel = 0.0

                p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
                final_waypoints.insert(0, p)

        return final_waypoints
    
    ### --- ###

    ### Publisher methods ###
    def publish_waypoints(self, final_waypoints):
        lane = Lane()
        lane.header = self.base_waypoints.header
        lane.waypoints = final_waypoints
        self.final_waypoints_pub.publish(lane)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')