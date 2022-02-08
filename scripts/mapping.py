#!/usr/bin/env python3

import rospy
import tf
import numpy as np
import copy
import yaml
import os
from tf2_ros.buffer_interface import convert 
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from tf2_geometry_msgs import PoseStamped
from Estimate import DEG_TO_RAD,RAD_TO_DEG, Estimate
from math import pi
from dynamic_reconfigure.server import Server
from riptide_mapping.cfg import MappingConfig

#TODO: DEBUGGING
#from tf.transformations import quaternion_from_euler, euler_from_quaternion

# Our overall data representation; each object has related information 
# We fill the publishers in __init__
objects = {
    "cutie": { # Old game object used for testing
        "pose": None, 
        "publisher" : None
    },
    "tommy": { # Tommygun game object.
        "pose":None,
        "publisher": None
    },
    "gman": { # GMan game object.
        "pose":None,
        "publisher": None
    },
    "bootlegger": { # Bootlegger game object.
        "pose":None,
        "publisher": None
    },
    "badge": { # Badge game object.
        "pose":None,
        "publisher": None
    },
    "gate": {
        "pose": None,
        "publisher" : None
    },
}

# Used to translate between DOPE ids and names of objects
object_ids = {
    0 : "gate",
    1 : "cutie", 
    2 : "tommy", 
    3 : "gman", 
    4 : "bootlegger", 
    5 : "badge", 
}

# Handles merging DOPE's output into our representation
# msg: Detection3DArray (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3DArray.html)
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
            # Translate the ID that DOPE gives us to a name meaningful to us
            name = object_ids[result.id]

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

            #TODO: DEBUGGING
            # msg = reading_world_frame
            # msg_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
            # _,_, msg_yaw = euler_from_quaternion(msg_quat) # just need yaw.

            # rospy.loginfo("************Raw detection yaw: {yaw}".format(yaw = msg_yaw * RAD_TO_DEG))

            reading_world_frame.pose.pose.orientation

            # We do some error/reasonability checking with this
            reading_camera_frame = PoseWithCovarianceStamped()
            reading_camera_frame.header.frame_id = cameraFrame
            reading_camera_frame.header.stamp = t 
            reading_camera_frame.pose = result.pose

            # Merge the given position into our position for that object
            objects[name]["pose"].addPositionEstimate(reading_world_frame, reading_camera_frame, result.score)

            # Publish that object's data out 
            output_pose = objects[name]["pose"].get_pose_with_covariance_stamped()
            objects[name]["publisher"].publish(output_pose)

            # Publish /tf data for the given object 
            pose = output_pose.pose.pose # Get the embedded geometry_msgs/Pose (we don't need timestamp/covariance)
            translation = (pose.position.x, pose.position.y, pose.position.z) # Needs to be a 3-tuple rather than an object
            rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) # Needs to be a 4-tuple rather than an object
            time = t
            child = name + "_frame"
            parent = "world"
            tb.sendTransform(translation, rotation, time, child, parent)

        

# Handles reconfiguration for the mapping system.
# NOTE: Reconfig reconfigures all values, not just the one specified in rqt.
def reconfigCallback(config, level):
    objectGroups = config["groups"]["groups"]["Objects"]["groups"]
    for objectName in objectGroups: # Checking EVERY object that could have been reconfigured.
        
        objectName = objectName.lower()

        # Get pose data from reconfig and update our map accordingly
        object_position = [config['{}_x_pos'.format(objectName)], config['{}_y_pos'.format(objectName)], config['{}_z_pos'.format(objectName)]]
        object_yaw = (config['{}_yaw'.format(objectName)]) * DEG_TO_RAD # Users will reconfig in degrees, system uses rads.
        object_covariance = [config['{}_x_cov'.format(objectName)], config['{}_y_cov'.format(objectName)], config['{}_z_cov'.format(objectName)], config['{}_yaw_cov'.format(objectName)]]
        
        # Create new estimate from reconfigured data.
        objects[objectName]["pose"] = Estimate(object_position, object_yaw, object_covariance)

        # Update filter 
        objects[objectName]["pose"].stdev_cutoff = config['stdev_cutoff']
        objects[objectName]["pose"].angle_cutoff = config['angle_cutoff']
        objects[objectName]["pose"].cov_limit = config['cov_limit']
        objects[objectName]["pose"].k_value = config['k_value']
        objects[objectName]["pose"].distance_limit = config['distance_limit']
        
        # Publish reconfigured data
        objects[objectName]["publisher"].publish(objects[objectName]["pose"].get_pose_with_covariance_stamped())

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
    tb = tf.TransformBroadcaster()
    worldFrame = "world"
    cameraFrame = "{}stereo/left_optical".format(rospy.get_namespace())

    # Creating publishers
    for field in objects:
        objects[field]["publisher"] = rospy.Publisher("mapping/" + field, PoseWithCovarianceStamped, queue_size=1)

    # Dynamic reconfiguration server
    server = Server(MappingConfig,reconfigCallback)

    # Load base variance file
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

    rate = rospy.Rate(0.5) # ROS Rate at 0.5Hz
    
    # Publish data for each object
    while not rospy.is_shutdown():
        for object_name in objects:    
            objects[object_name]["publisher"].publish(objects[object_name]["pose"].get_pose_with_covariance_stamped())
        rate.sleep()


    rospy.spin()
    
