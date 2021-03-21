#!/usr/bin/env python

import rospy
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TransformStamped
from gazebo_msgs.msg import ModelStates
from Pose import Pose
import tf

# TODO: Define our representation.
currentPositions = {
    "gate": Pose(), 
    "buoy": Pose(), 
    "markers": Pose(), 
    "torpedoes": Pose(), 
    "retrieve": Pose()
}

# Handles merging DOPE's output into our representation
# Takes a Detection3DArray in as the message
# Detection3DArray Docs @ http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3DArray.html
def dopeCallback(msg):
    return # Not doing anything for DOPE on this branch

# Handle /gazebo/model_states data
def gazeboCallback(msg):
    for i in range(len(msg.name)):

        # Filter out anything that isn't a prop we're interested in 
        if "world" in msg.name[i] or "lake" in msg.name[i] or "puddles" in msg.name[i]:
            continue

        # We assume what we have left is a prop and we will be publishing *something*
        obj = PoseWithCovarianceStamped()
        obj.pose.pose = msg.pose[i]
        now = rospy.get_rostime()
        obj.header.stamp.secs = now.secs
        obj.header.stamp.nsecs = now.nsecs

        # Actually publish, dependent on what the name was
        if msg.name[i] == "pole":
            polePub.publish(obj)
        elif msg.name[i] == "gate":
            gatePub.publish(obj)
        elif msg.name[i] == "cutie":
            cutiePub.publish(obj)
        elif msg.name[i] == "buoy":
            buoyPub.publish(obj)
        elif msg.name[i] == "markers":
            markersPub.publish(obj)
        elif msg.name[i] == "torpedoes":
            torpedoesPub.publish(obj)
        elif msg.name[i] == "retrieve":
            retrievePub.publish(obj)
        else:
            rospy.logerr("MAPPING ERROR: Tried to process something that isn't one of our props, but we haven't blacklisted! Object: " + msg.name[i])

        # Update tf
        translation = msg.pose[i].position
        translation = (translation.x, translation.y, translation.z) # Needs to be a 3-tuple rather than an object
        rospy.logdebug(translation)
        rotation = msg.pose[i].orientation
        rotation = (rotation.x, rotation.y, rotation.z, rotation.w) # Needs to be a 4-tuple rather than an object
        rospy.logdebug(rotation)
        time = rospy.get_rostime()
        child = msg.name[i] + "_frame"
        parent = "world"
        tf.TransformBroadcaster().sendTransform(translation, rotation, time, child, parent)

if __name__ == '__main__':

    rospy.init_node("mapping")

    # TODO: Read initial positions into whatever our representation is 

    # TODO: Register all our subscribers and publishers 
    # Subscribers
    rospy.Subscriber("/dope/detected_objects", Detection3DArray, dopeCallback)
    rospy.Subscriber("/gazebo/model_states", ModelStates, gazeboCallback)

    # Publishers
    gatePub = rospy.Publisher("mapping/gate", PoseWithCovarianceStamped, queue_size=1) # Gate task
    cutiePub = rospy.Publisher("mapping/cutie", PoseWithCovarianceStamped, queue_size=1) # Cutie
    buoyPub = rospy.Publisher("mapping/buoy", PoseWithCovarianceStamped, queue_size=1) # Buoy task 
    markersPub = rospy.Publisher("mapping/markers", PoseWithCovarianceStamped, queue_size=1) # Markers task
    torpedoesPub = rospy.Publisher("mapping/torpedoes", PoseWithCovarianceStamped, queue_size=1) # Manipulation/torpedoes task 
    retrievePub = rospy.Publisher("mapping/retrieve", PoseWithCovarianceStamped, queue_size=1) # Retrieve, surface, release task 
    polePub = rospy.Publisher("mapping/pole", PoseWithCovarianceStamped, queue_size=1) # Retrieve, surface, release task 

    rospy.spin()