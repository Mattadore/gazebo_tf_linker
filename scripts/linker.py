#!/usr/bin/env python
#this would be much better in C++ but I don't really wanna rewrite it
import rospy
import tf
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import LinkStates
from tf.transformations import *

listener = None
broadcaster = None
lastupdate = 0
#lastupdate = None

def onMessaged(linkstates):
    global listener
    global broadcaster
    global lastupdate
    if (rospy.get_rostime() - lastupdate).to_sec() < 0.03:
        return
    lastupdate = rospy.get_rostime()
    lastUpdateTime = rospy.get_rostime()
    for (link_idx, link_name) in enumerate(linkstates.name):
        robot_name = link_name.partition('::')[0]
        part_name = link_name.partition('::')[2]
        quat = linkstates.pose[link_idx].orientation
        pos = linkstates.pose[link_idx].position
        #rospy.loginfo(part_name)
        if listener.frameExists(part_name):
            #rospy.loginfo('stuff')
            broadcaster.sendTransform((pos.x,pos.y,pos.z),
                                      (quat.x,quat.y,quat.z,quat.w),
                                      rospy.get_rostime(),
                                      '/'+part_name,
                                      'world')
        if listener.frameExists(robot_name+'/'+part_name):
            #rospy.loginfo('otherstuff')
            broadcaster.sendTransform((pos.x,pos.y,pos.z),
                                      (quat.x,quat.y,quat.z,quat.w),
                                      rospy.get_rostime(),
                                      '/'+robot_name+'/'+part_name,
                                      'world')

def main():
  rospy.init_node('tf_linker_node')
  global lastupdate
  global broadcaster
  global listener
  lastupdate = rospy.get_rostime()
  broadcaster = tf.TransformBroadcaster()
  listener = tf.TransformListener()

  linkStatesSub = rospy.Subscriber('gazebo/link_states', LinkStates, onMessaged)

  rospy.spin()

if __name__ == '__main__':
  main()
