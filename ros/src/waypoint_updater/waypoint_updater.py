#!/usr/bin/env python
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

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
MAX_DECEL = 0.5 # Maximum deceleration factor. Can be tuned

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')
        
        # Initialize member variables
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoints_tree = None
        self.stopline_wp_idx =-1
        
        # Create subscribers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        # TODO: Add a subscriber for /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Call the loop to run periodically
        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints:
                #Publish waypoints
                self.publish_waypoints()
            rate.sleep()
     
    def get_closest_waypoint_idx(self):
        # Parse pose
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        
        # Find closest index by querying tree
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        
        # Check if closest waypoint is ahead or behind vehicle
        # Get closest and prev waypoint --> ref vector
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]
        
        # Plane through the coordinates as a dot product
        cl_vec = np.array(closest_coord)
        prev_vec = np.array(prev_coord)
        pos_vec = np.array([x, y])
        
        val = np.dot(cl_vec-prev_vec, pos_vec-cl_vec)
        
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        
        return closest_idx
    
    def publish_waypoints(self):
        # Call generate lane function to get waypoints and velocities
        final_lane = self.generate_lane()
        
        # Publish waypoints
        self.final_waypoints_pub.publish(final_lane)
    
    def generate_lane(self):
        # Create a lane message
        lane = Lane()
        
        # Assign base header, not really used
        lane.header = self.base_waypoints.header
        
        # Get closest and furthest waypoint indices
        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        
        # Splice waypoints from closest index to LOOKAHEAD_WPS points ahead / the end of waypoints
        base_waypoints = self.base_waypoints.waypoints[closest_idx : farthest_idx]
        
        # If stop line not found or beyond farthest point, return waypoints untouched, otherwise decelerate
        if (self.stopline_wp_idx == -1) or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)
        
        return lane
    
    def decelerate_waypoints(self, waypoints, closest_idx):
        tmp = []
        return temp
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            
            # Set stop index to two waypoints behind the stop line (approx half the length of the car)
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            dist = self.distance(waypoints, i, stop_idx)
            
            # Compute velocity as a square root function of distance
            vel = math.sqrt(2 * MAX_DECEL * dist) # Could be improved by a more continuous function that has a smooth derivative like an S-curve
            
            # Zero out small velocity values
            if vel < 1:
                vel = 0
            
            # Assign computed velocity subject to velocity (Speed) limit
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)
            
        return temp            
    
    def pose_cb(self, msg):
       self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)        

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
