#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import numpy as np
import copy
import statistics
import yaml
import os
from tf2_ros.buffer_interface import convert 
from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection3DArray
from tf.transformations import euler_from_quaternion
from tf2_geometry_msgs import PoseStamped

# The number of samples to collect before calculating the variances
maxSamples = 0

# Certainty threshold
minCertainty = 1.0

# Target we are doing a covariance calibration for
targetName = ""

# Used to translate between DOPE ids and names of objects
objectIDs = {
    0 : "gate",
    1 : "cutie", 
    2 : "buoy", 
    3 : "markers", 
    4 : "torpedoes", 
    5 : "retrieve", 
}

# global variables for runtime
samples = []

# ROS parameter name for the yaml file
fileName = os.path.join(os.path.dirname(os.path.dirname(os.path.realpath(__file__))), "cfg", "covariances.yaml")

print(fileName)

covarSaved = False

def detectionCallback(msg):
    global covarSaved
    if(len(samples) <= maxSamples):
        #print("got detect {}".format(msg))
        for detection in msg.detections:
            #print(detection)
            # Verify TF system is established
            try:
                (trans, rot) = tl.lookupTransform(worldFrame, cameraFrame, rospy.Time(0))
                t = tl.getLatestCommonTime(worldFrame, cameraFrame)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logwarn("TF check failed")
                return 

            # Note that we don't change the first loop to `detection in msg.detections.results` because we want the timestamp from the Detection3D object
            # Context: This loop will run <number of objects DOPE can identify> times 
            # `result` is of type ObjectHypothesisWithPose (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/ObjectHypothesisWithPose.html)
            for result in detection.results:

                # Translate the ID that DOPE gives us to a name meaningful to us
                name = objectIDs[result.id]

                # Save a new sample if it meets the critereia
                if(name == targetName and result.score >= minCertainty):

                    if(len(samples) % int(maxSamples / 10.0) == 0):
                        rospy.loginfo("Adding sample {} for {}".format(len(samples), name))

                    # convert from camera frame to world frame
                    p1 = PoseStamped()
                    p1.header.frame_id = cameraFrame
                    p1.header.stamp = t
                    p1.pose = result.pose.pose

                    convertedPos = tl.transformPose(worldFrame, p1)

                    samples.append(copy.deepcopy(convertedPos.pose))

    elif(not covarSaved):
        rospy.loginfo("Data collection complete.\nCalculating vision system variance")

        # We have enough data to run statistics now
        xData = []
        yData = []
        zData = []
        yawData = []

        #print(samples)

        # pool data into corresponding lists
        for sample in samples:
            xData.append(sample.position.x)
            yData.append(sample.position.y)
            zData.append(sample.position.z)

            # Convert quaternion to euler angle
            orient = [sample.orientation.x,sample.orientation.y ,sample.orientation.z, sample.orientation.w]
            roll, pitch, yaw = euler_from_quaternion(orient)
            yawData.append(yaw)

        # Calculate corresponding variances
        xVar = float(statistics.variance(xData))
        yVar = float(statistics.variance(yData))
        zVar = float(statistics.variance(zData))
        yawVar = float(statistics.variance(yawData))

        # Load exisitng yaml into dict
        covarianceData = {}
        with open(fileName, 'r') as fs:
            try:
                yamlData = yaml.safe_load(fs)
                if(not yamlData is None):
                    covarianceData = yamlData
                else:
                    rospy.logwarn("File did not contain any yaml")

            except Exception as e:
                rospy.logerror("Exception reading yaml file: {}".format(e))
                
        # Update covar data
        covarianceData[targetName] = {
            "x": xVar,
            "y": yVar,
            "z": zVar,
            "yaw": yawVar
        }

        rospy.loginfo("Updating mapping covariance data: {}" .format(covarianceData))

        # Save data back to yaml
        with open(fileName, 'w') as fs:
            yaml.safe_dump(covarianceData, fs, default_flow_style=False)

        covarSaved = True
        rospy.loginfo("Mapping system covariances updated")


# Main function
if __name__ == '__main__':

    rospy.init_node("mapping_covariance")

    if(rospy.get_namespace() == "/"):
        rospy.logwarn("Namespace may be incorrect, launched on {}".format(rospy.get_namespace()))

    # Handle startup parameters from tom of this file

    #print(rospy.get_param_names())

    targetName = rospy.get_param("~target", "cutie")
    maxSamples = int(rospy.get_param("~samples", 100))
    minCertainty = float(rospy.get_param("~certainty", 0.5))

    if(maxSamples < 10):
        rospy.logerror()

    # "class" variables 
    tl = tf.TransformListener()
    worldFrame = "world"
    cameraFrame = "{}stereo/left_optical".format(rospy.get_namespace())

    # Subscribers
    rospy.Subscriber("{}dope/detected_objects".format(rospy.get_namespace()), Detection3DArray, detectionCallback) # DOPE's information 

    rospy.loginfo("Computing mapping system variance for target '{}' from {} data points with minimum certainty of {}".format(targetName, maxSamples, minCertainty))

    rospy.spin()
    