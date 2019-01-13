#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
SPEEED_LIMIT_MPH = 40

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # set buffer size for /current_pose in order to avoid delay in receiving updated pose
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb, queue_size=1, buff_size=2**24)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = []
        self.current_pose = None
        self.traffic_wp_idx = None
        self.veh_wp_idx = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            # Publish the waypoints at 50 Hz
            if len(self.base_waypoints) > 0:
                [idx, wp] = self.closest_waypoint()
                self.veh_wp_idx = idx
                #rospy.loginfo('Pose = [%f,%f]; Closest Waypoint = [%f,%f]', self.current_pose[0], self.current_pose[1], self.base_waypoints[idx][0], self.base_waypoints[idx][1])
                if (idx is not None) and (wp is not None):
                    final_waypoint = Lane()
                    for i in range(LOOKAHEAD_WPS):
                        wp = Waypoint()
                        wp.pose.pose.position.x = self.base_waypoints[(idx+i) % len(self.base_waypoints)][0]
                        wp.pose.pose.position.y = self.base_waypoints[(idx+i) % len(self.base_waypoints)][1]
                        wp.pose.pose.position.z = self.base_waypoints[(idx+i) % len(self.base_waypoints)][2]
                        # update the speed later based on detected traffic lights and obstacles
                        wp.twist.twist.linear.x = self.base_waypoints[(idx+i) % len(self.base_waypoints)][3]
                        wp.twist.twist.linear.y = self.base_waypoints[(idx+i) % len(self.base_waypoints)][4]
                        wp.twist.twist.linear.z = self.base_waypoints[(idx+i) % len(self.base_waypoints)][5]
                        final_waypoint.waypoints.append(Waypoint(wp.pose, wp.twist))

                    self.final_waypoints_pub.publish(final_waypoint)

                    # rospy.loginfo('Wp[0] = [%f,%f]; Wp[end] = [%f,%f]',final_waypoint.waypoints[0].pose.pose.position.x,
                    #                                                     final_waypoint.waypoints[0].pose.pose.position.y,
                    #                                                     final_waypoint.waypoints[len(final_waypoint.waypoints)-1].pose.pose.position.x,
                    #                                                     final_waypoint.waypoints[len(final_waypoint.waypoints)-1].pose.pose.position.y)

            rate.sleep()

    def pose_cb(self, msg):
        #rospy.logwarn('callback from waypoint_updater: \n%s', msg.pose.position)
        self.current_pose = [msg.pose.position.x,
                            msg.pose.position.y,
                            msg.pose.position.z]

    def waypoints_cb(self, waypoints):
        rospy.loginfo('Received initial waypoints..')
        if not len(self.base_waypoints):
            for i in range(len(waypoints.waypoints)):
                self.base_waypoints.append([waypoints.waypoints[i].pose.pose.position.x,
                                            waypoints.waypoints[i].pose.pose.position.y,
                                            waypoints.waypoints[i].pose.pose.position.z,
                                            waypoints.waypoints[i].twist.twist.linear.x,
                                            waypoints.waypoints[i].twist.twist.linear.y,
                                            waypoints.waypoints[i].twist.twist.linear.z])

    def traffic_cb(self, msg):
        self.traffic_wp_idx = msg.data
        if self.traffic_wp_idx is not -1:
            # red light ahead
            dist_to_stop_line = self.distance(self.base_waypoints, self.veh_wp_idx, self.traffic_wp_idx)
            rospy.logwarn('Red Traffic Light ahead at distance %f', dist_to_stop_line)

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def mph_to_mps(self, mph):
        mps = mph*0.44704
        return mps

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2  + (a[2]-b[2])**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1], waypoints[i])
            wp1 = i
        return dist

    def closest_waypoint(self):
        idx = None
        waypoint = None
        min_dist = 100000
        if (self.base_waypoints is not None) and (self.current_pose is not None):
            for i in range(len(self.base_waypoints)):
                dist = self.euclidean_dist_3d(self.base_waypoints[i], self.current_pose)
                if(dist < min_dist):
                    idx = i
                    waypoint = self.base_waypoints[i]
                    min_dist = dist

            if idx is not None:
                # Check if the closest_waypoint is ahead of vehicle, if not then use idx+i
                closest_coord = self.base_waypoints[idx][0:1]
                prev_coord = self.base_waypoints[idx-1][0:1]
                # Equation for hyperplane through closest_coords
                cl_vect = np.array(closest_coord)
                prev_vect = np.array(prev_coord)
                pos_vect = np.array(self.current_pose[0:1])

                # dot product of (x1 + j.y1) and (x2 + j.y2) will be positive if they are in the same direction
                # If vectors (cl_vect-prev_vect) and (pos_vect-cl_vect) are in the same direction then pos_vect
                # will be ahead of cl_vect
                val = np.dot(cl_vect-prev_vect, pos_vect-cl_vect)
                if val > 0:
                    idx = (idx + 1) % len(self.base_waypoints)
                    waypoint = self.base_waypoints[idx]

        return [idx, waypoint]

    def euclidean_dist_3d(self, wp1, wp2):
        return math.sqrt((wp1[0]-wp2[0])**2 + (wp1[1]-wp2[1])**2  + (wp1[2]-wp2[2])**2)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
