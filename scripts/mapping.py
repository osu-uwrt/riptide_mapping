#!/usr/bin/env python

import rospy
import tf
import yaml
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from Pose import CustomPose

# Our overall data representation; each object has related information 
# We fill the publishers in __init__
objects = {
    "gate": {
        "pose": CustomPose(),
        "publisher": None
    },
    "buoy": {
        "pose": CustomPose(),
        "publisher": None
    },
    "markers": {
        "pose": CustomPose(),
        "publisher": None
    },
    "torpedoes": {
        "pose": CustomPose(),
        "publisher": None
    },
    "retrieve": {
        "pose": CustomPose(),
        "publisher": None
    }
}

# DOPE will likely give us a probability map, but it will be linked via IDs instead of names. This is how we translate.
# We might be able to just make this an array unless the IDs we get from dope are not sequential for some reason.
objectIDs = {
    0 : "gate", 
    1 : "buoy", 
    2 : "markers", 
    3 : "torpedoes", 
    4 : "retrieve"
}

# Handles merging DOPE's output into our representation
# msg: Detection3DArray (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3DArray.html)
def dopeCallback(msg):

    # Context: This loop will run <number of different objects DOPE thinks it sees on screen> times
    # `detection` is of type Detection3D (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3D.html)
    for detection in msg.detections:

        # Note that we don't change the first loop to `detection in msg.detections.results` because we want the timestamp from the Detection3D object
        # Context: This loop will run <number of objects DOPE can identify> times 
        # `result` is of type ObjectHypothesisWithPose (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/ObjectHypothesisWithPose.html)
        for result in detection.results: 


            # NOTE: Rather than decide here if it meets our threshold for adding an estimate, the cleaner way to do it is
            # to just pass it over regardless. addPositionEstimate will simply barely change our estimate if our certainty is low.

            # Translate the ID that DOPE gives us to a name meaningful to us
            name = objectIDs[result.id]

            # TODO: Convert DOPE's output into a PoseWithCovariancestamped object with positions in the *world frame*.
            # DOPE's frame is the same as the camera frame, specifically the left lens of the camera.
            convertedPos = 0

            # Merge the given position into our position for that object
            objects[name]["pose"].addPositionEstimate(convertedPos)

            # Publish that object's data out 
            poseStamped = objects[name]["pose"].getPoseWithCovarianceStamped()
            objects[name]["publisher"].publish(poseStamped)

            # Publish /tf data for the given object 
            pose = poseStamped.pose.pose # Get the embedded geometry_msgs/Pose (we don't need timestamp/covariance)
            translation = (pose.position.x, pose.position.y, pose.position.z) # Needs to be a 3-tuple rather than an object
            rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) # Needs to be a 4-tuple rather than an object
            time = rospy.get_rostime()
            child = name + "_frame"
            parent = "world"
            tf.TransformBroadcaster().sendTransform(translation, rotation, time, child, parent)


# Takes in an object and position data to produce an initial pose estimate 
# and associated covariance for the given object
# 
# param: object - object to get estimate of
# param: positions - parsed yaml object (ie yaml.load) of position data
# returns: newPose - CustomPose object created from the initial data
def initialObjectPose(object, positions):
    
    # Load object data from positions
    objectData = positions['objects']

    objectPosition = objectData[object]['pose']['position']
    objectOrientation = objectData[object]['pose']['orientation']
    
    # Instantiate a Pose object to store pose information
    objectPose = Pose()

    # Position data
    objectPose.position.x = objectPosition[0]
    objectPose.position.y = objectPosition[1]
    objectPose.position.z = objectPosition[2]

    # Orientation data
    objectPose.orientation.w = objectOrientation[0]
    objectPose.orientation.x = objectOrientation[1]
    objectPose.orientation.y = objectOrientation[2]
    objectPose.orientation.z = objectOrientation[3]

    # 6x6 covariance matrix
    objectCovariance = objectData[object]['covariance']
    
    # Instantiate a CustomPose object to store pose and covariance information
    newPose = CustomPose()
    newPose.pose = objectPose
    newPose.covariance = objectCovariance

    return newPose
    
if __name__ == '__main__':

    rospy.init_node("mapping")

    # Initial object positions loaded from positions.yaml

    initial_positions_file = open(rospy.get_param("/puddles/mapping/initial_positions"))
    initial_positions = yaml.load(initial_positions_file, Loader=yaml.FullLoader)
    
    # For each of our objects, set an initial estimate of their pose
    for object in objects:
        newPose = initialObjectPose(object, initial_positions)
        # Set this objects pose to the new one that was just created
        objects[object]['pose'] = newPose

        
    # Subscribers
    rospy.Subscriber("/dope/detected_objects", Detection3DArray, dopeCallback) # DOPE's information 

    # Publishers
    for field in objects:
        objects[field]["publisher"] = rospy.Publisher("/mapping/" + field, PoseWithCovarianceStamped, queue_size=1)

    rospy.spin()