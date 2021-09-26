#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import numpy as np
import copy
import yaml
import os
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

        # Verify TF system is established
        try:
            (trans, rot) = tl.lookupTransform(worldFrame, cameraFrame, rospy.Time(0))
            t = tl.getLatestCommonTime(worldFrame, cameraFrame)

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return 

        # Note that we don't change the first loop to `detection in msg.detections.results` because we want the timestamp from the Detection3D object
        # Context: This loop will run <number of objects DOPE can identify> times 
        # `result` is of type ObjectHypothesisWithPose (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/ObjectHypothesisWithPose.html)
        for result in detection.results: 

            print("Covariance{}".format(result.pose.covariance))
            # Translate the ID that DOPE gives us to a name meaningful to us
            name = objectIDs[result.id]

            # DOPE's frame is the same as the camera frame, specifically the left lens of the camera.
            # We need to convert that to the world frame, which is what is used in our mapping system 
            # Tutorial on how this works @ http://wiki.ros.org/tf/TfUsingPython#TransformerROS_and_TransformListener
            # Transform the pose part 
            p1 = PoseStamped()
            p1.header.frame_id = cameraFrame
            p1.header.stamp = t
            p1.pose = result.pose.pose

            convertedPos = tl.transformPose(worldFrame, p1)

            # Get the reading in the world frame message all together
            reading_world_frame = PoseWithCovarianceStamped()
            reading_world_frame.header.stamp = t
            reading_world_frame.header.frame_id = worldFrame
            reading_world_frame.pose.pose = copy.deepcopy(convertedPos.pose)

            # Apply rotation to covariance matrix 
            # To do that you just need to make a rotation matrix and then apply it to your covariance matrix
            # Applying a rotation to a matrix is R * COV * RT where the T is transpose

            # TODO: This is how covariance should be being calculated, however, DOPE is not giving us a covariance value for its detections.
            # covariance_matrix = np.array(result.pose.covariance).reshape((6, 6))
            # rotation_matrix = tf.transformations.quaternion_matrix(rot)[:3, :3]
            # cov = covariance_matrix[:3, :3] 
            # rotated_result = np.dot(rotation_matrix, np.dot(cov, rotation_matrix.T))
            # covariance_matrix[:3, :3] = rotated_result
            # reading_world_frame.pose.covariance = covariance_matrix.ravel()

            # We do some error/reasonability checking with this
            reading_camera_frame = PoseWithCovarianceStamped()
            reading_camera_frame.header.frame_id = cameraFrame
            reading_camera_frame.header.stamp = t 
            reading_camera_frame.pose = result.pose

            # Merge the given position into our position for that object
            objects[name]["pose"].addPositionEstimate(reading_world_frame, reading_camera_frame, result.score)

            # Publish that object's data out 
            output_pose = objects[name]["pose"].getPoseWithCovarianceStamped()
            objects[name]["publisher"].publish(output_pose)

            # Publish /tf data for the given object 
            pose = output_pose.pose.pose # Get the embedded geometry_msgs/Pose (we don't need timestamp/covariance)
            translation = (pose.position.x, pose.position.y, pose.position.z) # Needs to be a 3-tuple rather than an object
            rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) # Needs to be a 4-tuple rather than an object
            time = t
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
        objects[objectName]["pose"].setCovLimit(config['covLimit'])
        
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

# Handles the base object variance for each object.
def base_object_variance(object_name, data):
    target_data = data[object_name]
    variance = np.array([target_data['x'],target_data['y'],target_data['z'],target_data['yaw']], float)
    
    return variance


if __name__ == '__main__':

    rospy.init_node("mapping")
    
    # "class" variables 
    tl = tf.TransformListener()
    worldFrame = "world"
    cameraFrame = "{}stereo/left_optical".format(rospy.get_namespace())

    # Creating publishers
    for field in objects:
        objects[field]["publisher"] = rospy.Publisher("mapping/" + field, PoseWithCovarianceStamped, queue_size=1)

    # Dynamic reconfiguration server
    server = Server(MappingConfig,reconfigCallback)

    '''
    initial_data_file = open(rospy.get_param("~initial_object_data"))
    initial_data = yaml.load(initial_data_file, Loader=yaml.Loader)
    '''
    
    fileName = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "cfg", "covariances.yaml")
    initial_covariance = {}

    with open(fileName, 'r') as fs:
        try:
            yamlData = yaml.safe_load(fs)
            if(not yamlData is None):
                initial_covariance = yamlData
            else:
                rospy.logwarn("File did not contain any yaml")

        except Exception as e:
            rospy.logerror("Exception reading yaml file: {}".format(e))

    # Intitial base variance for each object 
    for object_name,_ in initial_covariance.items(): 
        objects[object_name]["pose"].base_variance = base_object_variance(object_name, initial_covariance)

    # Subscribers
    rospy.Subscriber("{}dope/detected_objects".format(rospy.get_namespace()), Detection3DArray, dopeCallback) # DOPE's information 

    rospy.spin()
    