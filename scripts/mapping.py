#!/usr/bin/env python

import rospy
import tf
import yaml
import numpy
from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import Detection3D
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
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
    }, 
    "pole": {
        "pose": CustomPose(), 
        "publisher": None
    }
}

# DOPE will likely give us a probability map, but it will be linked via IDs instead of names. This is how we translate.
# We might be able to just make this an array unless the IDs we get from dope are not sequential for some reason.
objectIDs = {
    0 : "pole",
    1 : "gate", 
    2 : "buoy", 
    3 : "markers", 
    4 : "torpedoes", 
    5 : "retrieve", 
}

# Handles merging DOPE's output into our representation
# msg: Detection3D (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3D.html)
def poleCallback(msg):

    # Verify TF system is established
    try:
        (trans, rot) = tl.lookupTransform(worldFrame, cameraFrame, rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return 

    # Note that we don't change the first loop to `detection in msg.detections.results` because we want the timestamp from the Detection3D object
    # Context: This loop will run <number of objects DOPE can identify> times 
    # `result` is of type ObjectHypothesisWithPose (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/ObjectHypothesisWithPose.html)
    for result in msg.results: 

        # NOTE: Rather than decide here if it meets our threshold for adding an estimate, the cleaner way to do it is
        # to just pass it over regardless. addPositionEstimate will simply barely change our estimate if our certainty is low.

        # Translate the ID that DOPE gives us to a name meaningful to us
        name = objectIDs[result.id]

        # DOPE's frame is the same as the camera frame, specifically the left lens of the camera.
        # We need to convert that to the world frame, which is what is used in our mapping system 
        # Tutorial on how this works @ http://wiki.ros.org/tf/TfUsingPython#TransformerROS_and_TransformListener
        # Transform the pose part 
        t = tl.getLatestCommonTime(worldFrame, cameraFrame)
        p1 = PoseStamped()
        p1.header.frame_id = cameraFrame
        p1.header.stamp = t
        p1.pose = result.pose.pose
        convertedPos = tl.transformPose(worldFrame, p1)

        p1Stamped = PoseWithCovarianceStamped()
        p1Stamped.header = p1.header 
        p1Stamped.pose.pose = convertedPos.pose

        # BUG: We transform a PoseStamped perfectly fine, but there's no easy way to transform the covariance, which is what gets actually fed into our system 
        # To do that you just need to make a rotation matrix and then apply it to your covariance matrix
        # Applying a rotation to a matrix is R * COV * RT where the T is transpose
        rotation_matrix = tf.transformations.quaternion_matrix(rot)[:3, :3]
        cov = numpy.array([ result.pose.covariance[0:3], result.pose.covariance[6:9], result.pose.covariance[12:15] ])
        rotated_result = numpy.dot(rotation_matrix, numpy.dot(cov, rotation_matrix.T))
        p1Stamped.pose.covariance[0:3] = rotated_result[0, 0:3] # Translation gets rotated
        p1Stamped.pose.covariance[6:9] = rotated_result[1, 0:3]
        p1Stamped.pose.covariance[12:15] = rotated_result[2, 0:3]
        p1Stamped.pose.covariance[5:6] = result.pose.covariance[5:6] # Rotation doesn't get rotated

        # poseMsg = PoseStamped()
        # poseMsg.header = detection.header # tf transforms need a PoseStamped, not a PoseWithCovarianceStamped like we have
        # poseMsg.pose = result.pose.pose
        # convertedPos = tl.transformPose("world", pose)

        # Merge the given position into our position for that object
        objects[name]["pose"].addPositionEstimate(p1Stamped, result.score)

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

# Handles merging DOPE's output into our representation
# msg: Detection3DArray (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3DArray.html)
def dopeCallback(msg):
    pass

    '''

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

            # DOPE's frame is the same as the camera frame, specifically the left lens of the camera.
            # We need to convert that to the world frame, which is what is used in our mapping system 
            # Tutorial on how this works @ http://wiki.ros.org/tf/TfUsingPython#TransformerROS_and_TransformListener
            worldFrame = "/world"
            cameraFrame = "{}stereo/left_link".format(rospy.get_namespace())
            convertedPos = None
            if transformer.frameExists(worldFrame) and transformer.frameExists(cameraFrame):
                t = tl.getLatestCommonTime(worldFrame, cameraFrame)
                rospy.logwarn_throttle(1, "t: " + str(t))
                p1 = PoseStamped()
                p1.header.frame_id = cameraFrame
                p1.header.stamp = t
                p1.pose = result.pose.pose
                rospy.logwarn_throttle(1, "p1: " + str(p1))
                convertedPos = tl.transformPose(worldFrame, p1)
            else:
                rospy.logerr_throttle(1, "ERROR: Mapping was unable to find /world and {}stereo/left_link frames!".format(rospy.get_namespace()))


            # poseMsg = PoseStamped()
            # poseMsg.header = detection.header # tf transforms need a PoseStamped, not a PoseWithCovarianceStamped like we have
            # poseMsg.pose = result.pose.pose
            # convertedPos = tl.transformPose("world", pose)

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
'''


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

    # "class" variables 
    tl = tf.TransformListener()
    worldFrame = "/world"
    cameraFrame = "{}stereo/left_link".format(rospy.get_namespace())

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
    rospy.Subscriber("pole_detection", Detection3D, poleCallback)

    # Publishers
    for field in objects:
        objects[field]["publisher"] = rospy.Publisher("mapping/" + field, PoseWithCovarianceStamped, queue_size=1)

    rospy.spin()