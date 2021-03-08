from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from operator import add, div
import rospy 
import numpy 

# Custom pose class that has commonly used mapping functionality 
class CustomPose:

    def __init__(self):
        self.pose = Pose()
        self.covariance = [0] * 36 # float64[36] type
        self.confidence = [0, 0, 0, 0] # 4-tuple representing x-y-z-yaw confidence        
    
    # Calculates a weighted average between the current value and a value from a message.
    # Weight is calculated from covariance. Also calculates a new covariance value.
    # param: currVal - current value stored in this object
    # param: msgVal - value received from a ROS message
    # param: currCovarianceVal - current covariance for this value
    # param: msgCovarianceVal - covariance of the value from the message
    # returns: newVal - new value calculated from the paramters above
    # returns: newCov - new covariance value 
    # Takes in one axis's data; that's one pose value for each and one covariance *row* for each 
    def compareValues (self, currVal, msgVal, currConfidence, msgConfidence):

        currWeight = currConfidence * 100
        msgWeight = msgConfidence * 100

        print("Current Weight: {}".format(currWeight))
        print("Message Weight: {}".format(msgWeight))

        newVal = ((currVal * currWeight) + (msgVal * msgWeight)) / (currWeight + msgWeight)

        # Calculate a new covariance value by essentially averaging both arrays 
        # newCov = list(map(add, currCovarianceVal, msgCovarianceVal))
        # newCov = [x / 2 for x in newCov]
        # newCov = 

        return newVal # , newCov

    # Takes in a reading and returns False if it's an invalid measurement we want to filter out, True otherwise
    # msg: PoseWithCovarianceStamped 
    # TODO: Implement
    def isValidMeasurement(self, msg, confidence):

        # Reject anything that isn't at least a moderate confidence value (also gets rid of division by zero errors down the line)
        if confidence < .05:
            print("Rejecting due to low confidence ({}).".format(confidence))
            return False

        # Reject anything that isn't realistically inside transdec
        if msg.pose.pose.position.x >= 500 or msg.pose.pose.position.x <= -500 or \
            msg.pose.pose.position.x >= 500 or msg.pose.pose.position.x <= -500 or \
            msg.pose.pose.position.x >= 500 or msg.pose.pose.position.x <= -500:
            print("Rejecting due to being too far from the origin.")
            return False

        # Reject anything that isn't reasonably flat 
        quat = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
        roll, pitch, _ = euler_from_quaternion(quat)
        if roll <= -15 or roll >= 15 or \
            pitch <= -15 or roll >= 15:
            print("Rejecting due to having abnormal roll/pitch.")
            return False 

        # TODO: Reject anything that is too far from the robot for us to realistically perceive (maybe 100m?) 
        # TODO: Reject anything that isn't within the robot's realistic field of view 
        # TODO: If we're at a high confidence rating, reject anything that is significantly far from our current reading 

        # If it didn't fail any checks, assume it's a good reading 
        return True 

    # Takes in a new DOPE reading and updates our estimate
    # msg: PoseWithCovarianceStamped Object (http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovariance.html)
    # Confidence: [0, 1] score of how confident DOPE is that it is the object in question 
    def addPositionEstimate(self, msg, confidence):

        # Filter out "invalid" measurements 
        if not self.isValidMeasurement(msg, confidence):
            return

        # Convenient aliases
        # currPose = self.pose 
        # currCovariance = self.covariance 
        msgPose = msg.pose.pose
        msgCovariance = msg.pose.covariance

        print("addPositionEstimate()")
        print("Current Pose: {}".format(self.pose))
        # print("Current Covariance: {}".format(self.covariance))
        print("Message Pose: {}".format(msgPose))
        # print("Message Covariance: {}".format(msgCovariance))

        # Convert angles to euler
        _, _, currYaw = euler_from_quaternion([self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z])
        _, _, msgYaw = euler_from_quaternion([msgPose.orientation.w, msgPose.orientation.x, msgPose.orientation.y, msgPose.orientation.z])

        # Update poses
        self.pose.position.x = self.compareValues(self.pose.position.x, msgPose.position.x, self.confidence[0], confidence) # x
        self.pose.position.y = self.compareValues(self.pose.position.y, msgPose.position.y, self.confidence[1], confidence) # y
        self.pose.position.z = self.compareValues(self.pose.position.z, msgPose.position.z, self.confidence[2], confidence) # z
        newYaw = self.compareValues(currYaw, msgYaw, self.confidence[3], confidence) # yaw

        # TODO: Update Covariances... it's something like this but there's probably a lot wrong with it 
        # self.pose.position.x, self.covariance[0:6] = self.compareValues(self.pose.position.x, msgPose.position.x, self.covariance[0:6], msgCovariance[0:6]) # x
        # self.pose.position.y, self.covariance[6:12] = self.compareValues(self.pose.position.y, msgPose.position.y, self.covariance[6:12], msgCovariance[6:12]) # y
        # self.pose.position.z, self.covariance[12:18] = self.compareValues(self.pose.position.z, msgPose.position.z, self.covariance[12:18], msgCovariance[12:18]) # z

        # Convert angles back to quaternions and update result  
        self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z = quaternion_from_euler(0, 0, newYaw)

        print("Ending Pose: {}".format(self.pose))
        # print("Ending Covariance: {}".format(self.covariance))
        
    # Compiles our representation into an object that we want 
    # return: PoseWithCovarianceStamped message (http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
    def getPoseWithCovarianceStamped(self):
        obj = PoseWithCovarianceStamped()
        obj.pose.pose = self.pose
        obj.pose.covariance = self.covariance
        obj.header.stamp = rospy.get_rostime() # Time Stamp
        return obj
