#!/usr/bin/env python
import rospy
import tf2_ros
import time
from nav_msgs.msg import OccupancyGrid # For map
from geometry_msgs.msg import Vector3, PoseStamped, PointStamped
from std_msgs.msg import Empty

# Jennifer Long
# Note: I was not able to complete this project due to many deadlines from other classes
# Here is what I have set up so far:
# 1) Created a new node called rescue.py in the simple_control package and added it to the launch file
# 2) Added subscribers and publishers on the specified topics
# 3) Set up logic in mainloop to fetch hiker position and call path planner (through publisher)
# 4) Set up logic to move drone directly to hiker when hiker reports exact location (within 5m)


# Take hiker position and return goal
class Rescue():

  def __init__(self):
    time.sleep(10)
    # Last updated position of hiker
    self.hiker_pos = None

    # Init map variables
    self.map = []
    self.width = -1
    self.height = -1
    self.origin_x = 0
    self.origin_y = 0

    # Init the drone and goal position
    self.drone_position = []
    self.hiker_last_pos
    self.hiker_exact_pos

    # Init variables used by the trajectory
    self.optimal_height = 25.0 # Needs above 20 to see tower

    # Init variables used to control the main loop
    self.descending = False
    self.within_close_range = False
    self.hiker_moved = False

    # Subscribers
    self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)
    # Subscribe to hiker position on map from tower_to_map
    self.hiker_pos_sub = rospy.Subscriber('/hiker/position', Vector3, self.get_hiker_pos, queue_size = 1)
    self.hiker_exact_pos_sub = rospy.Subscriber('/hiker/exact/position', Vector3, self.get_exact_hiker_pos, queue_size = 1)

    # Publishers
    self.goal_pub = rospy.Publisher('/uav/input/goal', Vector3, queue_size=1)
    self.cancel_traj_pub = rospy.Publisher('cancel/trajectory', Empty, queue_size = 1)
    self.position_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)

    # start main loop
    self.mainloop()

  # Callback: Get the map
  def get_map(self, msg):
    # Get the map width and height
    self.width = msg.info.width
    self.height = msg.info.height
    # Get the drone position
    self.origin_x = msg.info.origin.position.x
    self.origin_y = msg.info.origin.position.y
    # Get the map
    self.map = np.reshape(msg.data, (self.width, self.height))

  # Callback: Get the GPS data
  def get_gps(self, msg):
    self.drone_position = [int(round(msg.pose.position.x - self.origin_x, 0)), int(round(msg.pose.position.y- self.origin_y, 0))]

  # Callback for the hiker position subscriber
  def get_hiker_pos(self, msg):
    self.hiker_last_pos = msg

  # Callback for the hiker position subscriber
  def get_hiker_exact_pos(self, msg):
    self.hiker_exact_pos = msg
    self.within_close_range = True

  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():

      # Within close range
      if not self.descending and self.within_close_range:
        self.descending = True
        # Has exact position from hiker
        if self.hiker_exact_pos:
            # Move drone to waypoint
            msg = Vector3()
            msg.x = hiker_exact_pos.x
            msg.y = hiker_exact_pos.y
            msg.z = hiker_exact_pos.z
            self.position_pub.publish(msg)

      # Has estimated position from tower
      elif self.hiker_last_pos:
        
        # Plan obstacle-avoidant course from drone to hiker
        rospy.loginfo(str(rospy.get_name()) + ": Searching for hiker at {}".format([msg.x, msg.y]))
        self.goal_pub.publish(msg)

        # Reset the goal
        self.hiker_last_pos = None
        
      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('rescue')
  try:
    rescue = Rescue()
  except rospy.ROSInterruptException:
    pass