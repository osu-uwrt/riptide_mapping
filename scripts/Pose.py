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
        self.pose = Pose() # Pose Type
        self.covariance = [0] * 36 # float64[36] type
    
    # Calculates a weighted average between the current value and a value from a message.
    # Weight is calculated from covariance. Also calculates a new covariance value.
    # param: currVal - current value stored in this object
    # param: msgVal - value received from a ROS message
    # param: currCovarianceVal - current covariance for this value
    # param: msgCovarianceVal - covariance of the value from the message
    # returns: newVal - new value calculated from the paramters above
    # returns: newCov - new covariance value 
    # Takes in one axis's data; that's one pose value for each and one covariance *row* for each 
    def compareValues (self, currVal, msgVal, currCovarianceVal, msgCovarianceVal):

        # Make weight values from covariance values
        currWeight = 100 - max(currCovarianceVal) * 100
        msgWeight = 100 - max(msgCovarianceVal) * 100

        newVal = ((currVal * currWeight) + (msgVal * msgWeight)) / (currWeight + msgWeight)

        # Calculate a new covariance value by essentially averaging both arrays 
        newCov = list(map(add, currCovarianceVal, msgCovarianceVal))
        newCov = [x / 2 for x in newCov]

        return newVal, newCov

    # Takes in a reading and returns False if it's an invalid measurement we want to filter out, True otherwise
    # msg: PoseWithCovarianceStamped 
    # TODO: Implement
    def isValidMeasurement(self, msg):

        # Reject anything that isn't realistically inside transdec
        if msg.pose.pose.position.x >= 500 or msg.pose.pose.position.x <= -500 or \
            msg.pose.pose.position.x >= 500 or msg.pose.pose.position.x <= -500 or \
            msg.pose.pose.position.x >= 500 or msg.pose.pose.position.x <= -500:
            return False

        # Reject anything that isn't reasonably flat 
        roll, pitch, _ = euler_from_quaternion(msg.pose.pose.orientation)
        if roll <= -15 or roll >= 15 or \
            pitch <= -15 or roll >= 15:
            return False 

        # TODO: Reject anything that is too far from the robot for us to realistically perceive (maybe 100m?) 
        # TODO: Reject anything that isn't within the robot's realistic field of view 
        # TODO: If we're at a high confidence rating, reject anything that is significantly far from our current reading 

        # If it didn't fail any checks, assume it's a good reading 
        return True 

    # Takes in a new DOPE reading and updates our estimate
    # msg: PoseWithCovarianceStamped Object (http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovariance.html)
    def addPositionEstimate(self, msg):

        # Filter out "invalid" measurements 
        if not self.isValidMeasurement(msg):
            return

        # Convenient aliases
        # currPose = self.pose 
        # currCovariance = self.covariance 
        msgPose = msg.pose.pose
        msgCovariance = msg.pose.covariance

        print("addPositionEstimate()")
        print("Current Pose: {}".format(self.pose))
        print("Current Covariance: {}".format(self.covariance))
        print("Message Pose: {}".format(msgPose))
        print("Message Covariance: {}".format(msgCovariance))

        # Update translational poses/covariances
        self.pose.position.x, self.covariance[0:6] = self.compareValues(self.pose.position.x, msgPose.position.x, self.covariance[0:6], msgCovariance[0:6]) # x
        self.pose.position.y, self.covariance[6:12] = self.compareValues(self.pose.position.y, msgPose.position.y, self.covariance[6:12], msgCovariance[6:12]) # y
        self.pose.position.z, self.covariance[12:18] = self.compareValues(self.pose.position.z, msgPose.position.z, self.covariance[12:18], msgCovariance[12:18]) # z

        # Update rotational poses/covariance (NOTE: We set roll/pitch to zero, all we care about is yaw)
        # Convert Quaternion -> Euler, update yaw, then convert Euler->Quaternion
        _, _, currYaw = euler_from_quaternion([self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z])
        _, _, msgYaw = euler_from_quaternion([msgPose.orientation.w, msgPose.orientation.x, msgPose.orientation.y, msgPose.orientation.z])
        newYaw, self.covariance[30:36] = self.compareValues(currYaw, msgYaw, self.covariance[30:36], msgCovariance[30:36])
        self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z = quaternion_from_euler(0, 0, newYaw)

        print("Ending Pose: {}".format(self.pose))
        print("Ending Covariance: {}".format(self.covariance))
        
    # Compiles our representation into an object that we want 
    # return: PoseWithCovarianceStamped message (http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
    def getPoseWithCovarianceStamped(self):
        obj = PoseWithCovarianceStamped()
        obj.pose.pose = self.pose
        obj.pose.covariance = self.covariance
        obj.header.stamp = rospy.get_rostime() # Time Stamp
        return obj
