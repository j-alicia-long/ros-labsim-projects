#!/usr/bin/env python
import rospy
import numpy as np
import copy

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Empty
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import PoseStamped, Pose

import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from math import sin, cos, degrees, sqrt
from tf.transformations import euler_from_quaternion


class Viewer():

  def __init__(self):
    # When this node shutsdown
    rospy.on_shutdown(self.shutdown_sequence)

    # Set the rate
    self.rate = 1
    self.dt = 1.0 / self.rate

    # Create the position
    self.drone_current = Pose()

    # Create the dimensions
    self.start_x = -5
    self.start_y = -5
    self.end_x = 5
    self.end_y = 5

    # Create the trajectory
    self.traj = np.array([[0, 0]])

    # Create the viewer
    self.fig = plt.figure()
    self.ax = self.fig.gca(projection='3d')
    self.ax.set_xlim([self.start_x, self.end_x])
    self.ax.set_ylim([self.start_y, self.end_y])
    self.ax.set_zlim([0, 5])
    self.ax.set_title("Pos: (" + str(round(0)) + ", " + str(round(0)) + ", " + str(round(0)) + ")")
    self.scat, = self.ax.plot([0], [0], [0], marker=".", linestyle="", markersize=15, color='r')
    self.line, = self.ax.plot(self.traj[:,0], self.traj[:,1], np.full(len(self.traj[:,1]), 2.5), marker=".", linestyle="--", markersize=10, color='g')
    self.ax.set_xlabel('X Axis')
    self.ax.set_ylabel('Y Axis')
    self.ax.set_zlabel('Z Axis')
    plt.ion() 
    plt.show()

    # Keep track if we need to update the axis
    self.updateaxis = False

    # Keep track of the obstacles
    self.obstacle_list = []
    
    # Create the subscribers and publishers
    self.gps_sub = rospy.Subscriber("uav/sensors/gps", PoseStamped, self.get_gps)

    # Subscribe to the map data
    self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.get_map)

    # Collision publisher
    self.col_pub = rospy.Publisher('/uav/collision', Empty, queue_size=1)

    # Subscribe to the trajectory
    self.trajectory_sub = rospy.Subscriber('/uav/trajectory', Int32MultiArray, self.get_traj)

    # Run the communication node
    self.DrawLoop()


  # Call back to get the gps data
  def get_gps(self, msg):
    self.drone_current.position = msg.pose.position
    self.drone_current.orientation = msg.pose.orientation

  def get_traj(self, msg):
    self.traj = np.array(np.reshape(msg.data, (-1, 2)))
    self.traj[:, 0] = self.traj[:, 0]
    self.traj[:, 1] = self.traj[:, 1]

# Call back to get the gps data
  def get_map(self, msg):

    self.updateaxis = True
    
    width = msg.info.width
    height = msg.info.height
    res = msg.info.resolution

    self.start_x = msg.info.origin.position.x
    self.start_y = msg.info.origin.position.y

    self.end_x = self.start_x + (width * res)
    self.end_y = self.start_y + (height * res)

    map_data = np.reshape(msg.data, (width, height))
    
    for xi in range(0, width):
      for yi in range(0, height):

        if map_data[xi, yi] > 50:
          self.obstacle_list.append((xi + self.start_x, yi + self.start_y))


  # This is the main loop of this class
  def DrawLoop(self):
    # Set the rate
    rate = rospy.Rate(self.rate)

    # While running
    while not rospy.is_shutdown():

        # Display the position
        self.view_point()

        # Sleep any excess time
        rate.sleep()


	# Called on ROS shutdown
  def shutdown_sequence(self):
    rospy.loginfo(str(rospy.get_name()) + ": Shutting Down")


  def insideObstacle(self):
    inside = False
    for obs in self.obstacle_list:
      if (obs[0] < self.drone_current.position.x < obs[0] + 1) and (obs[1] < self.drone_current.position.y < obs[1] + 1):
        inside = True
        break

    return inside


  def view_point(self):
    euler = euler_from_quaternion([
      self.drone_current.orientation.x,
      self.drone_current.orientation.y,
      self.drone_current.orientation.z,
      self.drone_current.orientation.w])
    roll = euler[0]
    pitch = euler[1]
    yaw = euler[2]
    new_x = sin(yaw)
    new_y = cos(yaw)

    x = self.drone_current.position.x
    y = self.drone_current.position.y
    z = self.drone_current.position.z
    
    redraw_all = False
    draw_height = 0

    # Update the title
    self.ax.title.set_text("Pos: (" + str(round(x,2)) + ", " + str(round(y,2)) + ", " + str(round(z,2)) + ")")

    # Check if we need to update the xlim and ylim
    if self.updateaxis:
      self.ax.set_xlim([self.start_x, self.end_x])
      self.ax.set_ylim([self.start_y, self.end_y])
      self.ax.set_zlim([0, 5])

    # Draw the quiver and scatter and keep track of where they are in the collections
    #draw_position = len(self.ax.collections)
    #self.ax.quiver(x, y, z, new_x, new_y, 0, color='r')
    self.scat.set_xdata([x])
    self.scat.set_ydata([y])
    self.scat.set_3d_properties([z])
    self.line.set_xdata(self.traj[:,0])
    self.line.set_ydata(self.traj[:,1])
    self.line.set_3d_properties(np.full(len(self.traj[:,1]), 2.5))

    # Only draw obstacles if they have not been drawn before
    if len(self.ax.collections) < len(self.obstacle_list):
      draw_height = self.drone_current.position.z
      # Draw obstacles
      for obs in self.obstacle_list:
        obs_x = obs[0]
        obs_y = obs[1]
        x_obs = [obs_x, obs_x + 1, obs_x + 1, obs_x]
        y_obs = [obs_y, obs_y, obs_y + 1, obs_y + 1]
        z_obs = [draw_height, draw_height, draw_height, draw_height]
        verts = [list(zip(x_obs, y_obs, z_obs))]
        self.ax.add_collection3d(Poly3DCollection(verts))
        redraw_all = True

    # Check for collisions
    if self.insideObstacle():
      rospy.loginfo("Collision Detected!!!")
      self.col_pub.publish(Empty())
      rospy.signal_shutdown("collision")

    # Draw the plot
    if redraw_all:
      self.fig.canvas.draw()
      self.fig.canvas.flush_events()
    else:
      self.ax.draw_artist(self.ax.patch)
      self.ax.draw_artist(self.scat)
      self.ax.draw_artist(self.line)
      #self.fig.canvas.update()
      self.fig.canvas.flush_events()      

    for spine in self.ax.spines.values(): self.ax.draw_artist(spine)

    # Remove the obstacles if the drone has changed height significantly
    if abs(draw_height - z) > 0.5:
      # while len(self.ax.collections) > 0:
      self.ax.collections = []



def main():
  rospy.init_node('viewer_node')
  try:
    v = Viewer()
  except rospy.ROSInterruptException:
    pass


if __name__ == '__main__':
  main()
