#!/usr/bin/env python
import rospy
import copy
import numpy as np
from astar_class import AStarPlanner
from geometry_msgs.msg import Vector3
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid

# Create a class which we will use to take keyboard commands and convert them to a position
class PathPlanner():
  # On node initialization
  def __init__(self):
    # Create the publisher and subscriber
    self.position_pub = rospy.Publisher('/uav/input/position', Vector3, queue_size=1)
    self.trajectory_pub = rospy.Publisher('/uav/trajectory', Int32MultiArray, queue_size=1)
    self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)
    self.requested_position = rospy.Subscriber("/uav/input/goal", Vector3, self.get_goal)
    self.atwaypoint_sub = rospy.Subscriber('uav/sensors/atwaypoint', Bool, self.at_waypoint, queue_size = 1)

    # Initialize class variables
    self.width = -1
    self.height = -1
    self.drone_position = []
    self.goal_position = []
    self.have_plan = False
    self.map = []
    self.at_waypoint = False

    # Call the mainloop of our class
    self.mainloop()

  # Map callback
  def get_map(self, msg):
  
    # Get the map width and height
    self.width = msg.info.width
    self.height = msg.info.height

    # Get the drone position
    self.origin_x = msg.info.origin.position.x
    self.origin_y = msg.info.origin.position.y
    self.drone_position = [int(self.width + self.origin_x), int(self.height + self.origin_y)]

    # Get the map
    self.map = np.reshape(msg.data, (self.width, self.height))

    # Reset plan
    self.have_plan = False

  # Callback for the keyboard manager
  def at_waypoint(self, msg):
    # Save the drones alitude
    self.at_waypoint = msg.data

  # Goal callback
  def get_goal(self, msg):
  
    if len(self.goal_position) == 0:
      # Get the goal position
      x = int(round(msg.x, 0) - self.origin_x)
      y = int(round(msg.y, 0) - self.origin_y)

      # Get the drone position
      self.goal_position = [x, y]
      
      # Reset plan
      self.have_plan = False

  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(3)

    # Checks if the plan has been started
    self.have_plan = False
    sent_position = False

    # Create the trajectory publish message
    p_traj = Int32MultiArray()
    current_waypoint = Vector3()

    # While ROS is still running
    while not rospy.is_shutdown():

      # If you dont have a plan wait for a map, current position, and a goal
      if not self.have_plan:
        # If we have received the data
        if (len(self.map) != 0) and (len(self.drone_position) == 2) and (len(self.goal_position) == 2):
          rospy.loginfo(str(rospy.get_name()) + ": Planning trajectory")
          astar = AStarPlanner(safe_distance=1)
          trajectory = astar.plan(self.map, self.drone_position, self.goal_position)
          if trajectory != None:
            trajectory = np.array(trajectory)
            self.have_plan = True
            trajectory[:, 0] = trajectory[:, 0] + self.origin_x
            trajectory[:, 1] = trajectory[:, 1] + self.origin_y
            rospy.loginfo(str(rospy.get_name()) + ": Executing trajectory")   
            rospy.loginfo(str(rospy.get_name()) + ": " + str(trajectory)) 
          else:
            rospy.loginfo(str(rospy.get_name()) + ": Trajectory not found, try another goal")  
      # We have a plan, execute it
      else:    

        # Publish the trajectory
        if len(p_traj.data) != len(np.reshape(trajectory,-1)):
          p_traj.data = np.reshape(trajectory,-1)
          self.trajectory_pub.publish(p_traj)

        # Publish the current waypoint
        if self.at_waypoint == False or sent_position == False or np.shape(trajectory)[0] < 0:
          msg = Vector3()
          msg.x = trajectory[0][0]
          msg.y = trajectory[0][1]
          msg.z = 2.5
          self.position_pub.publish(msg)
          sent_position = True
        else:
          trajectory = trajectory[1:]
          sent_position = False

        # If we are done wait for next goal
        if np.shape(trajectory)[0] <= 0 and self.at_waypoint:
          self.have_plan = False
          self.drone_position = copy.deepcopy(self.goal_position)
          self.goal_position = []
          continue

      # Sleep for the remainder of the loop
      rate.sleep()


if __name__ == '__main__':
  rospy.init_node('path_planning_node')
  try:
    pp = PathPlanner()
  except rospy.ROSInterruptException:
    pass