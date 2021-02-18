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

    # Iterate through each object DOPE has provided us
    for detection in msg.detections:
        
        # Parses through the results of a detection and determines which object has been detected from an ID associated with a confidence score
        objectID = 0
        largestScore = 0

        for result in detection.results:
            resultID = result.id
            resultScore = result.score

            # If this score is more confident than the last, then set its corresponding ID to the object ID
            if (resultScore > largestScore):
                largestScore = resultScore
                objectID = resultID
        
        name = objectIDs[objectID]

        # name is set to "gate" here because the code above  may not work without the proper DOPE information and object IDs
        name = "gate"

        # TODO: Convert DOPE's position into world position

        # Merge the given position into our position for that object
        objects[name]["pose"].addPositionEstimate(detection)

        # Publish that object's data out 
        poseStamped = objects[name]["pose"].getPoseWithCovarianceStamped()
        objects[name]["publisher"].publish(poseStamped)

        # Publish /tf data for the given object 
        # TODO: This *should* work, but hasn't been tested; test it!
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
    objectPose.position = objectPosition
    objectPose.orientation = objectOrientation
   
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