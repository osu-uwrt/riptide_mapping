#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import numpy as np
import copy
from tf2_ros.buffer_interface import convert 
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from tf2_geometry_msgs import PoseStamped
from Estimate import Estimate
from math import pi

from dynamic_reconfigure.server import Server
from riptide_mapping.cfg import MappingConfig

# Our overall data representation; each object has related information 
# We fill the publishers in __init__
objects = {

    "cutie": {
        "pose": None,
        "publisher" : None
    },
    "gate": {
        "pose": None,
        "publisher" : None
    },

    "pole":{
        "pose": None,
        "publisher" : None

    }
}

# Used to translate between DOPE ids and names of objects
objectIDs = {
    0 : "gate",
    1 : "cutie", 
    2 : "buoy", 
    3 : "markers", 
    4 : "torpedoes", 
    5 : "retrieve", 
}

# Handles merging DOPE's output into our representation
# msg: Detection3DArray (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3DArray.html)
# TODO: We know pole_callback works. Adjust this to work. Should be as easy as copy-pasting pole_callback into the loop in here, as pole_callback takes in a Detection3D.
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
        
            # DOPE's frame is the same as the camera frame, specifically the left lens of the camera.
            # We need to convert that to the world frame, which is what is used in our mapping system 
            # Tutorial on how this works @ http://wiki.ros.org/tf/TfUsingPython#TransformerROS_and_TransformListener
            worldFrame = "world"
            cameraFrame = "puddles/stereo/left_link"

            convertedPose = None
            if tf_buffer.can_transform(worldFrame,cameraFrame,rospy.Time()):
                t = rospy.Time(0)
                rospy.logwarn_throttle(1, "t: " + str(t))
                p1 = PoseStamped() # tf2 geometry message
                p1.header.frame_id = cameraFrame
                p1.header.stamp = t
                p1.pose = result.pose.pose
                rospy.logwarn_throttle(1, "p1: " + str(p1))
                convertedPose = tf_buffer.transform(p1, worldFrame)
                
            else:
                rospy.logerr_throttle(1, "ERROR: Mapping was unable to find /world and {}stereo/left_link frames!".format(rospy.get_namespace()))
            
            # Get covariance from score
            msg_covariance = scoreToCov(result.score)

            # Reconstruct a pose with covariance stamped message from transformed pose and original covariance
            object_message = PoseWithCovarianceStamped()
            object_message.header.frame_id = worldFrame
            object_message.header.stamp = rospy.Time.now()
            object_message.pose.pose = convertedPose.pose

            object_message.pose.covariance[0] = msg_covariance[0] # X covariance
            object_message.pose.covariance[7] = msg_covariance[1] # Y covariance
            object_message.pose.covariance[14] = msg_covariance[2] # Z covariance
            object_message.pose.covariance[35] = msg_covariance[3] # Yaw covariance

            rospy.loginfo("Message Covariance: {}".format(result.pose.covariance))

            # Used for error checking
            reading_camera_frame = PoseWithCovarianceStamped()
            reading_camera_frame.header.frame_id = cameraFrame
            reading_camera_frame.header.stamp = rospy.Time.now()
            reading_camera_frame.pose.pose = result.pose.pose
            reading_camera_frame.pose.covariance = result.pose.covariance

            score = result.score

            # Merge the given position into our position for that object
            objects[name]["pose"].addPositionEstimate(object_message, reading_camera_frame, score)

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

# Handles reconfiguration for the mapping system.
# TODO: In the future, we should only update the initial estimates of objects that have been reconfigured instead of 
# indiscriminately updating everything regardless of whether or not they were actually reconfigured.
def reconfigCallback(config, level):

    objectGroups = config["groups"]["groups"]["Objects"]["groups"]
    for objectName in objectGroups: # Checking EVERY object that could have been reconfigured.
        objectName = objectName.lower()

        # Get pose data from reconfig and update our map accordingly
        object_position = [config['{}_x_pos'.format(objectName)], config['{}_y_pos'.format(objectName)], config['{}_z_pos'.format(objectName)]]
        object_yaw = config['{}_yaw'.format(objectName)]
        object_covariance = [config['{}_x_cov'.format(objectName)], config['{}_y_cov'.format(objectName)], config['{}_z_cov'.format(objectName)], config['{}_yaw_cov'.format(objectName)]]
        objects[objectName]["pose"] = Estimate(object_position, object_yaw, object_covariance)

        # Update filter 
        objects[objectName]["pose"].setStdevCutoff(config['stdevCutoff'])
        objects[objectName]["pose"].setAngleCutoff(config['angleCutoff'])
        
        # Publish reconfigured data
        objects[objectName]["publisher"].publish(objects[objectName]["pose"].getPoseWithCovarianceStamped())

        # Console output (uncomment for debugging)
        '''
        rospy.loginfo("Position for {object} has been reconfigured: {newPos}".format(object = objectName, newPos = object_position))
        rospy.loginfo("Position for {object} has been reconfigured: {newPos}".format(object = objectName, newPos = object_position))
        rospy.loginfo("Yaw for {object} has been reconfigured: {newYaw}".format(object = objectName, newYaw = object_yaw))
        rospy.loginfo("Covariance for {object} has been reconfigured: {newCov}".format(object = objectName, newCov = object_covariance))
        '''

    # Console output (uncomment for debugging)
    '''
    rospy.loginfo("Standard deviation cutoff has been reconfigured: {stdevCutoff}".format(stdevCutoff = config['stdevCutoff']))
    rospy.loginfo("Angle cutoff has been reconfigured: {angleCutoff}".format(angleCutoff = config['angleCutoff']))
    '''
    return config

# Very simple conversion from DOPE score to covariance
def scoreToCov(score):
    cov = 1 - score # The lower the covariance, the more sure the system is. Basically just invert the score value
    covariance = [cov, cov, cov, cov] # x, y , z, yaw
    return covariance
if __name__ == '__main__':

    rospy.init_node("mapping")
    
    # "class" variables 
    tf_buffer = tf2_ros.Buffer()
    tl = tf2_ros.TransformListener(tf_buffer)

    # Creating publishers
    for field in objects:
        objects[field]["publisher"] = rospy.Publisher("mapping/" + field, PoseWithCovarianceStamped, queue_size=1)
        
    # Subscribers
    rospy.Subscriber("{}dope/detected_objects".format(rospy.get_namespace()), Detection3DArray, dopeCallback) # DOPE's information 

    # Dynamic reconfiguration server
    server = Server(MappingConfig,reconfigCallback)

    rospy.spin()
    
   

    