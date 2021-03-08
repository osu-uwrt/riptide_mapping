from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import sqrt
import rospy 
import numpy 

# Custom pose class that has commonly used mapping functionality 
class CustomPose:

    # TODO: Probably cleaner to just use a PoseWithCovarianceStamped object rather than keeping the fields separate 
    def __init__(self):
        self.pose = Pose()
        self.covariance = [0] * 36 # float64[36] type
        self.confidence = [0, 0, 0, 0] # 4-tuple representing x-y-z-yaw confidence   
        self.time = None      
    
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

        if currConfidence > 1 or currConfidence < 0:
            rospy.logerr("currConfidence bad value: {}".format(currConfidence))
        elif msgConfidence > 1 or msgConfidence < 0:
            rospy.logerr("msgConfidence bad value: {}".format(msgConfidence))

        # print("Current Weight: {}".format(currWeight))
        # print("Message Weight: {}".format(msgWeight))

        newVal = ((currVal * currConfidence) + (msgVal * msgConfidence)) / (currConfidence + msgConfidence)
        
        # Closer values increase our certainty, further values decrease our certainty 
        # TODO: Scale this based on how close to the object we are 
        certaintyChange = 0
        diff = abs(newVal - currVal)
        if diff < .1 and diff >= .025:
            certaintyChange = .000001 * (1 - currConfidence) * msgConfidence
        elif diff < .025 and diff >= .01:
            certaintyChange = .00001 * (1 - currConfidence) * msgConfidence
        elif diff < .01:
            certaintyChange = .0001 * (1 - currConfidence) * msgConfidence

        # Inaccurate, so 
        elif diff > .1 and diff <= .5:
            certaintyChange = -.00001 * (1 - currConfidence) * msgConfidence
        elif diff > .5:
            certaintyChange = -.0001 * (1 - currConfidence) * msgConfidence

        rospy.loginfo_throttle(1, "certaintyChange: {}".format(certaintyChange))

        # Calculate a new covariance value by essentially averaging both arrays 
        # newCov = list(map(add, currCovarianceVal, msgCovarianceVal))
        # newCov = [x / 2 for x in newCov]
        # newCov = 

        return newVal, certaintyChange # , newCov

    # Takes in a reading and returns False if it's an invalid measurement we want to filter out, True otherwise
    # msg: PoseWithCovarianceStamped 
    # TODO: Implement
    def isValidMeasurement(self, msg, confidence):

        # Used throughout
        msg_quat = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
        msg_roll, msg_pitch, msg_yaw = euler_from_quaternion(msg_quat)
        self_quat = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
        self_roll, self_pitch, self_yaw = euler_from_quaternion(self_quat)

        ''' 
        Confidence check temporarily removed, as the pole detector gives us zero for this. Normal perception won't; we'll test it further down the line.
        # Reject anything that isn't at least a moderate confidence value (also gets rid of division by zero errors down the line)
        if confidence < .1:
            print("Rejecting due to low confidence ({}).".format(confidence))
            return False
        '''

        # Reject anything that isn't realistically inside transdec or our environment 
        if msg.pose.pose.position.x >= 500 or msg.pose.pose.position.x <= -500 or \
            msg.pose.pose.position.y >= 500 or msg.pose.pose.position.y <= -500 or \
            msg.pose.pose.position.z >= 500 or msg.pose.pose.position.z <= -500:
            rospy.logwarn("Rejecting due to being too far from the origin.")
            return False

        # Reject anything that isn't reasonably flat 
        if msg_roll <= -15 or msg_roll >= 15 or \
            msg_pitch <= -15 or msg_pitch >= 15:
            rospy.logwarn("Rejecting due to having abnormal roll/pitch.")
            return False 

        # If we're at a high confidence rating, reject anything that is significantly far from our current reading
        # The concern here is that we get zeroed in on outliers too quickly; 
        # generally, outliers will be inconsistent enough that this isn't a concern, but it's still something to potentially worry about
        if self.confidence[0] > .9:
            diff = abs(self.pose.position.x - msg.pose.pose.position.x)
            if diff > 1:
                rospy.logwarn("Rejecting because our x-certainty {} is high enough to reject {}, which is {}m away.".format(self.confidence[0], msg.pose.pose.position.x, diff))
                return False 
        if self.confidence[1] > .9:
            diff = abs(self.pose.position.y - msg.pose.pose.position.y)
            if diff > 1:
                rospy.logwarn("Rejecting because our y-certainty {} is high enough to reject {}, which is {}m away.".format(self.confidence[1], msg.pose.pose.position.y, diff))
                return False 
        if self.confidence[2] > .9:
            diff = abs(self.pose.position.z - msg.pose.pose.position.z)
            if diff > 1:
                rospy.logwarn("Rejecting because our z-certainty {} is high enough to reject {}, which is {}m away.".format(self.confidence[2], msg.pose.pose.position.z, diff))
                return False 
        if self.confidence[3] > .9:
            diff = abs(self_yaw - msg_yaw)
            if diff > 45:
                rospy.logwarn("Rejecting because our yaw-certainty {} is high enough to reject {}deg, which is {}deg away.".format(self.confidence[3], msg_yaw, diff))
                return False 

        # TODO: Reject anything that is too far from the robot for us to realistically perceive (maybe 100m?) 
        # TODO: Reject anything that isn't within the robot's realistic field of view 
        # (which perception shouldn't give us in the first place, but doesn't hurt to check)

        # If it didn't fail any checks, assume it's a good reading 
        return True 

    # Takes in a new DOPE reading and updates our estimate
    # msg: PoseWithCovarianceStamped Object (http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovariance.html)
    # Confidence: [0, 1] score of how confident DOPE is that it is the object in question 
    def addPositionEstimate(self, msg, confidence):

        # Debug 
        rospy.loginfo_throttle(1, "Certainties: ({}%, {}%, {}%, {}%)" \
            .format(self.confidence[0] * 100, self.confidence[1] * 100, self.confidence[2] * 100, self.confidence[3] * 100))

        # Filter out "false positives"
        # If we make it past this, we assume it's a true positive and is actually the object
        if not self.isValidMeasurement(msg, confidence):
            return

        # Update timestamp so it's the most recent routine 
        self.time = rospy.get_rostime()

        # Convenient aliases
        msgPose = msg.pose.pose
        msgCovariance = msg.pose.covariance

        # print("addPositionEstimate()")
        # print("Current Pose: {}".format(self.pose))
        # print("Current Covariance: {}".format(self.covariance))
        # print("Message Pose: {}".format(msgPose))
        # print("Message Covariance: {}".format(msgCovariance))

        # Convert angles to euler
        _, _, currYaw = euler_from_quaternion([self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z])
        _, _, msgYaw = euler_from_quaternion([msgPose.orientation.w, msgPose.orientation.x, msgPose.orientation.y, msgPose.orientation.z])

        # Normalize the covariance array to [0, 1)
        if max(msgCovariance) != 0:
            for i in range(len(msgCovariance)):
                msgCovariance[i] = msgCovariance[i] / max(msgCovariance)
                msgCovariance[i] = (msgCovariance[i] + 1) / 2

        # Update poses
        self.pose.position.x, x_certaintychange = self.compareValues(self.pose.position.x, msgPose.position.x, self.confidence[0], 1 - msgCovariance[0]) # x
        self.pose.position.y, y_certaintychange = self.compareValues(self.pose.position.y, msgPose.position.y, self.confidence[1], 1 - msgCovariance[7]) # y
        self.pose.position.z, z_certaintychange = self.compareValues(self.pose.position.z, msgPose.position.z, self.confidence[2], 1 - msgCovariance[14]) # z
        newYaw, yaw_certaintychange = self.compareValues(currYaw, msgYaw, self.confidence[3], 1 - msgCovariance[35]) # yaw

        # Apply our certainty changes
        self.confidence[0] = x_certaintychange
        self.confidence[1] += y_certaintychange
        self.confidence[2] += z_certaintychange
        self.confidence[3] += yaw_certaintychange

        # Clamp to [0, 1)
        self.confidence[0] = max(0, min(self.confidence[0], 1))
        self.confidence[1] = max(0, min(self.confidence[0], 1))
        self.confidence[2] = max(0, min(self.confidence[0], 1))
        self.confidence[3] = max(0, min(self.confidence[0], 1))

        # TODO: Update Covariances... it's something like this but there's probably a lot wrong with it 
        # self.pose.position.x, self.covariance[0:6] = self.compareValues(self.pose.position.x, msgPose.position.x, self.covariance[0:6], msgCovariance[0:6]) # x
        # self.pose.position.y, self.covariance[6:12] = self.compareValues(self.pose.position.y, msgPose.position.y, self.covariance[6:12], msgCovariance[6:12]) # y
        # self.pose.position.z, self.covariance[12:18] = self.compareValues(self.pose.position.z, msgPose.position.z, self.covariance[12:18], msgCovariance[12:18]) # z

        # Convert angles back to quaternions and update result  
        self.pose.orientation.w, self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z = quaternion_from_euler(0, 0, newYaw)

        # print("Ending Pose: {}".format(self.pose))
        # print("Ending Covariance: {}".format(self.covariance))
        
    # Compiles our representation into an object that we want 
    # return: PoseWithCovarianceStamped message (http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
    def getPoseWithCovarianceStamped(self):
        obj = PoseWithCovarianceStamped()
        obj.pose.pose = self.pose
        obj.pose.covariance = self.covariance
        obj.header.stamp = self.time # Time Stamp
        return obj
