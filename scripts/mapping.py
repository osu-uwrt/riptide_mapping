#!/usr/bin/env python

import rospy
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from Pose import Pose

# TODO: Define our representation. Something like this will probably work, where the six-tuple is xyzrpy
currentPositions = {
    "gate": Pose, 
    "buoy": Pose, 
    "markers": Pose, 
    "torpedoes": Pose, 
    "retrieve": Pose
}

# Handles merging DOPE's output into our representation
# Takes a Detection3DArray in as the message
# Detection3DArray Docs @ http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3DArray.html
def dopeCallback(msg):

    # TODO: Convert DOPE's positions to world positions
    # Iterate through Detection3DArray to get individual Detection3D objects
    # Detection3D Docs @ http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3D.html
    for detectionMsg in msg:
        bbox = detectionMsg.bbox # Type: http://docs.ros.org/en/noetic/api/vision_msgs/html/msg/BoundingBox3D.html
        

    # TODO: Merge the changes from DOPE into our representation
    # Will need to do a little math to figure out the best way to update it.
    # We don't want to straight-up overwrite it every time and ignore all our previous data points, but we also know new data is more accurate so we want to use it.

    # TODO: Publish every object's relevant points 
    gatePub.publish()

''' Kept as an example of what a callback that does some processing looks like, can be deleted once we get our feet under us 
def bboxCb(msg):
    objects = {}
    for b in msg.bounding_boxes:
        if not objects.has_key(b.Class) or b.probability > objects[b.Class].probability:
            objects[b.Class] = b
    msg.bounding_boxes = objects.values()
    bboxPub.publish(msg)
'''

if __name__ == '__main__':

    rospy.init_node("mapping")

    # TODO: Read initial positions into whatever our representation is 

    # TODO: Register all our subscribers and publishers 
    # Subscribers
    rospy.Subscriber("/dope/detected_objects", Detection3DArray, dopeCallback)

    # Publishers
    gatePub = rospy.Publisher("mapping/gate", PoseWithCovarianceStamped, queue_size=1) # Gate task
    buoyPub = rospy.Publisher("mapping/buoy", PoseWithCovarianceStamped, queue_size=1) # Buoy task 
    markersPub = rospy.Publisher("mapping/markers", PoseWithCovarianceStamped, queue_size=1) # Markers task
    torpedoesPub = rospy.Publisher("mapping/torpedoes", PoseWithCovarianceStamped, queue_size=1) # Manipulation/torpedoes task 
    retrievePub = rospy.Publisher("mapping/retrieve", PoseWithCovarianceStamped, queue_size=1) # Retrieve, surface, release task 

    rospy.spin()