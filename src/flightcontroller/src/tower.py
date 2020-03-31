#!/usr/bin/env python
import rospy
import tf2_ros
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import OccupancyGrid
from tf2_geometry_msgs import do_transform_point

from enum import Enum
import numpy as np

class State(Enum):
    NO_TRAJ = 0
    FIRST_TRAJ = 1
    FIRST_ACK = 2
    FIRST_DONE = 3
    SECOND_TRAJ = 4
    SECOND_ACK = 5
    DONE = 6

class Tower:
    

    def __init__(self):

        rospy.loginfo("Initializing tower")
        
        # Obstacle publisher
        self.position_pub = rospy.Publisher('/obstacles', Int32MultiArray, queue_size=1)
        self.map_pub = rospy.Publisher('/map_update', Int32MultiArray, queue_size=1)

        # map variables
        self.map = None
        self.map_origin = None 

        # Map subscriber
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)

        self.state = State.NO_TRAJ
        #self.traj = None
        # Trajectory listener
        traj_sub = rospy.Subscriber('/uav/trajectory', Int32MultiArray, self.handle_traj)
        
        # transform listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        # start main loop
        self.mainloop()

    def handle_traj(self, msg):
        new_traj = np.reshape(np.array(msg.data), (-1,2))
        if self.state == State.NO_TRAJ and len(msg.data) > 0:
            self.state = State.FIRST_TRAJ
        if self.state == State.FIRST_ACK:
            if new_traj:
                #check if the new traj is a subset of the old one
                for cell in new_traj:
                    if not np.isin(self.traj):
                        self.state = State.SECOND_TRAJ
                        break
            else:
                self.state = State.SECOND_TRAJ
        rospy.loginfo("state {} trajectory: {}".format(self.state, new_traj))
            
    # Map callback
    def get_map(self, msg):
        # Get map transform
        self.map_origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        # Get the map
        self.map = np.reshape(msg.data, (msg.info.width, msg.info.height))

        
    def generate_obstacles(self, size):
        MAX_COUNT = 2 * size
        count = 0
        crt_size = 0
        obstacles = []

        if self.state == State.FIRST_TRAJ:
            pos = np.random.randint(self.traj.shape[0]/2, self.traj.shape[0])
            if pos < self.traj.shape[0]:
                obstacles.append(self.traj[pos])
        
        while count < MAX_COUNT and len(obstacles) < size:
            # choose a random position
            i = np.random.randint(0, self.map.shape[0])
            j = np.random.randint(0, self.map.shape[1])
            if self.map[i][j] == 0 and not np.isin( np.array([i,j], self.traj) ):
                #unocupied in map coordinates
                obstacles.append([i + int(self.map_origin[0]), j + int(self.map_origin[1])])
            count += 1
        return obstacles

    def do_transform(self, occupied, transform):
        result = []
        for c in occupied:
            point = PointStamped()
            point.point.x = c[0]
            point.point.y = c[1]
            new_point = do_transform_point(point, transform)
            result.append([int(new_point.point.x), int(new_point.point.y)])
        return result
        
    def mainloop(self):

        # Set the rate of this loop
        rate = rospy.Rate(1)

        # the message with occupied cells
        map_msg = Int32MultiArray()
        msg = Int32MultiArray()    
        while not rospy.is_shutdown():
            rospy.loginfo("Main loop")
            if self.state == State.FIRST_TRAJ or self.state == State.SECOND_TRAJ:
                try:
                    transform = self.tfBuffer.lookup_transform('world', 'tower', rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    rate.sleep()
                    rospy.loginfo("Transform not found! Waiting!")
                    continue
            
                occupied = self.generate_obstacles(10)
                new_occupied = self.do_transform(occupied, transform)
                rospy.loginfo("Occupied {}".format(occupied))
                rospy.loginfo("New {}".format(new_occupied))

                # send to visualizer
                map_msg.data = np.array(occupied).flatten()
                self.map_pub.publish(map_msg)
                # send to planner
                msg.data = np.array(new_occupied).flatten()
                self.position_pub.publish(msg);
                rospy.loginfo("Published")
                if self.state == State.FIRST_TRAJ:
                    self.state = State.FIRST_ACK
                else:
                    self.state = State.DONE
            # Sleep for the remainder of the loop
            rate.sleep()

            
if __name__ == '__main__':
    rospy.init_node('tower')
    try:
        pp = Tower()
    except rospy.ROSInterruptException:
        pass
    
