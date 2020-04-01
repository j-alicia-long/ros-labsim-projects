#!/usr/bin/env python
import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32MultiArray
from nav_msgs.msg import OccupancyGrid
from tf2_geometry_msgs import do_transform_point

from enum import Enum
import numpy as np


class Tower:
    

    def __init__(self):

        rospy.loginfo("Initializing tower")
        
        # Goal publisher
        self.goal_pub = rospy.Publisher("/tower/goal", Vector3, queue_size=1)

        # map variables
        self.map = None
        self.map_origin = None 
        # Map subscriber
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)

        self.empty_traj = True
        traj_sub = rospy.Subscriber('/uav/trajectory', Int32MultiArray, self.handle_traj)
        
        # transform listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        # start main loop
        self.mainloop()

    def handle_traj(self, msg):
        if len(msg.data) == 0:
            #rospy.loginfo("empty trajectory")
            self.empty_traj = True
        else:
            #rospy.loginfo("trajectory {}".format(np.reshape(msg.data, (-1,2))))
            self.empty_traj = False

    # Map callback
    def get_map(self, msg):
        # Get map transform
        self.map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        # Get the map
        self.map = np.reshape(msg.data, (msg.info.width, msg.info.height))

        
    def generate_goal(self):
        MAX_COUNT = self.map.shape[0] * self.map.shape[1]
        count = 0
        while count < MAX_COUNT:
            # choose a random position
            i = np.random.randint(0, self.map.shape[0])
            j = np.random.randint(0, self.map.shape[1])
            if self.map[i][j] == 0:
                return [i + self.map_origin[0], j + self.map_origin[1]]
            count += 1
        return None

    def do_transform(self, pos, transform):
        point = PointStamped()
        point.point.x = pos[0]
        point.point.y = pos[1]
        point.point.z = 0.0
        new_point = do_transform_point(point, transform)
        return [int(new_point.point.x), int(new_point.point.y)]
        
    def mainloop(self):

        # Set the rate of this loop
        rate = rospy.Rate(0.5)
        while not rospy.is_shutdown():

            if self.empty_traj:
                try:
                    transform = self.tfBuffer.lookup_transform('world', 'tower', rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rate.sleep()
                    rospy.loginfo("Transform not found! Waiting!")
                    continue
        
                new_goal = self.generate_goal()
                if new_goal:
                    new_gol = self.do_transform( new_goal, transform)
                    # send to planner
                    self.goal_pub.publish(Vector3(new_goal[0], new_goal[1], 0.0))
                    rospy.loginfo("Published goal {}".format(new_goal))
                    
            # Sleep for the remainder of the loop
            rate.sleep()

            
if __name__ == '__main__':
    rospy.init_node('tower')
    try:
        pp = Tower()
    except rospy.ROSInterruptException:
        pass
    
