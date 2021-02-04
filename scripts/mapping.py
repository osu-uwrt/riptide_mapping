#!/usr/bin/env python

import rospy
import tf
import yaml
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseWithCovarianceStamped
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
                objectID = resultid
        
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



def makeInitialEstimate(object, positions):
    
    # Format of pose array : (x,y,z,pitch,roll,yaw)
    pose = positions[object]['pose']

    # Format of size array : (width,depth,height)
    size = positions[object]['size']

    # Float between 0 and 1
    covariance = positions[object]['covariance']

    # Instantiate new pose
    newPose = CustomPose()
    newPose.pose = pose
    newPose.covariance = covariance
    newPose.size = size

    # Set this objects pose to the new one we just created
    objects[object]['pose'] = newPose
    
if __name__ == '__main__':

    rospy.init_node("mapping")

    # Initial object positions

    # Load information from config fil
    
    intial_positions_file = open(rospy.get_param('~initial_positions'))
    intial_positions = yaml.load(intial_positions_file, Loader=yaml.FullLoader)
    object_position_data = intial_positions['objects']
    
    # For each of our objects, set an initial estimate of their pose
    for object in objects:
        makeInitialEstimate(object, object_position_data)

        # TODO: Test if this loop does what its supposed to. It should, but it still needs properly tested.

    

    # Subscribers
    rospy.Subscriber("/dope/detected_objects", Detection3DArray, dopeCallback) # DOPE's information 

    # Publishers
    for field in objects:
        objects[field]["publisher"] = rospy.Publisher("/mapping/" + field, PoseWithCovarianceStamped, queue_size=1)

    rospy.spin()