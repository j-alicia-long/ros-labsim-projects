#!/usr/bin/env python
import rospy
import tf2_ros
import time
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PointStamped
from tf2_geometry_msgs import do_transform_point

class TowerToMap:

  def __init__(self):
    time.sleep(10)
    # Used by the callback for the topic /tower/goal
    self.goal = None

    # TODO: Instantiate the Buffer and TransformListener
    self.tfBuffer = tf2_ros.Buffer()
    self.listener = tf2_ros.TransformListener(self.tfBuffer)

    # TODO: Goal publisher on topic /uav/input/goal
    # self.goal_pub = rospy.Publisher('/uav/input/goal', Vector3, queue_size=1)
    self.goal_pub = rospy.Publisher('/hiker/position', Vector3, queue_size=1)
    
    # TODO: Tower goal subscriber to topic /tower/goal
    # self.tower_goal_sub = rospy.Subscriber('/tower/goal', Vector3, self.get_tower_goal, queue_size = 1)
    self.tower_goal_sub = rospy.Subscriber('/tower/hiker/position', Vector3, self.get_tower_goal, queue_size = 1)

    # start main loop
    self.mainloop()

  #TODO: Callback for the tower goal subscriber
  def get_tower_goal(self, msg):
    self.goal = msg

  def mainloop(self):
    # Set the rate of this loop
    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
      if self.goal:
        
        try: 
          #TODO: Lookup the tower to world transform
          transform = self.tfBuffer.lookup_transform('world', 'tower', rospy.Time())
          
          #TODO: Convert the goal to a PointStamped
          point = PointStamped()
          point.point.x = self.goal.x
          point.point.y = self.goal.y
          point.point.z = self.goal.z

          #TODO: Use the do_transform_point function to convert the point using the transform
          new_point = do_transform_point(point, transform)

          #TODO: Convert the point back into a vector message containing integers
          msg = Vector3()
          msg.x = new_point.point.x
          msg.y = new_point.point.y
          msg.z = new_point.point.z

          #TODO: Publish the vector
          rospy.loginfo(str(rospy.get_name()) + ": Publishing Transformed Goal {}".format([msg.x, msg.y]))
          self.goal_pub.publish(msg)

          # The tower will automatically send you a new goal once the drone reaches the requested position.
          #TODO: Reset the goal
          self.goal = None

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
          continue
        
      rate.sleep()

if __name__ == '__main__':
  rospy.init_node('tower_to_map')
  try:
    tom = TowerToMap()
  except rospy.ROSInterruptException:
    pass