from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import rospy 

# Custom pose class that has commonly used mapping functionality 
class CustomPose:

    def __init__(self):
        self.pose = Pose() # Pose Type
        self.covariance = [] # float64[36] type 


    ''' 
        TODO: Figure out how a weighted update should work, then implement it.
        The general idea is to give more weight to newer results, as uncertainty builds up over time.
        More weight should also be given to results where DOPE gives us a high confidence rating for it.
        Past that, not really sure exactly how this should work. There are likely established ways to do it
        (Dr. Z mentioned Potential Field & Gradient Descent) but odds are it'll have to be mostly custom and experimental.

        Two possible systems I could think of.

        #1 
        - Keep an array of every single PoseWithCovarianceStamped message we've gotten.
        - Every time getPoseWithCovarianceStamped() gets called, iterate through all our results and arrive at a new confidence value. 
        - Ways to optimize: Save only results >= a confidence level, only update our actual estimate once or twice a second 
        - Advantages: Very flexible. You always have all our data available.
        - Disadvantages: More processing power, *especially* on long runs
            - Note that we will be getting like 5-10 readings per second from DOPE maximum, so this isn't actually too bad.
        
        #2 
        - We store a single position and certainty. Probably also an indicator of how long we've been running, like # of readings above a certain threshold or total runtime.
        - Each "update" we do changes this one number.
        - Advantages: Near-zero processing/storage footprint; tuning is less complicated.
        - Disadvantages: No access to old data, meaning it's a little less flexible.

        I personally think we should do the second, then move to the first if it proves too unreliable, but I'm open to changing it.
        They can both give more precedence to newer results, and can both account for how sure DOPE itself is, so both are probably decent.
    '''
    
    # Calculates a weighted average between the current value and a value from a message.
    # Weight is calculated from covariance. Also calculates a new covariance value.
    # param: currVal - current value stored in this object
    # param: msgVal - value received from a ROS message
    # param: currCovarianceVal - current covariance for this value
    # param: msgCovarianceVal - covariance of the value from the message
    # returns: newVal - new value calculated from the paramters above
    # returns: newCov - new covariance value 
    def compareValues (self, currVal, msgVal, currCovarianceVal, msgCovarianceVal):

        # Make weight values from covariance values
        currWeight = currCovarianceVal * 100
        msgWeight = msgCovarianceVal * 100

        newVal = ((currVal * currWeight) + (msgVal * msgWeight)) / (currWeight + msgWeight)

        # Calculate a new covariance value
        newCov = (currCovarianceVal + msgCovarianceVal)/2

        return newVal, newCov

    # Takes in a new DOPE reading and updates our estimate
    # msg: PoseWithCovarianceStamped Object (http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovariance.html)
    def addPositionEstimate(self, msg):
        # Get the current pose and covariance and store it
        currPose = Pose() 
        currPose = self.pose
        currCovariance = self.covariance

        # Get the message pose and covariance store it
        msgPose = Pose()
        msgPose = msg.pose
        msgCovariance = msg.covariance

        # New pose and covariance to store everything
        newPose = Pose()
        newCovariance = []
 
        positionKeys = ["x","y","z"]

        positionCovarianceIndeces = {
            "x" : 0,
            "y" : 7,
            "z" : 14
        }

        # For every key, calculate and set a new position value for the new pose
        for key in positionKeys:
            # Covariance index for this key
            covarianceIndex = positionCovarianceIndeces[key]

            # Current position value
            currVal = getattr(currPose.position, key)
            currCovarianceVal = currCovariance[covarianceIndex]

            # Message position value
            msgVal = getattr(msgPose.position, key)
            msgCovarianceVal = msgCovariance[covarianceIndex]
             
            # Calculate a new value and covariance from these two
            newVal, newCov = self.compareValues(currVal,msgVal,currCovarianceVal,msgCovarianceVal)
            
            # Set these values to the corresponding values in newPose and newCovariance
            setattr(newPose.position, key, newVal)
            newCovariance[covarianceIndex] = newCov
            
        
        orientationCovarianceIndeces = {
            "Roll" : 21,
            "Pitch" : 28,
            "Yaw" : 35,
        }

        currOrientation = currPose.orientation
        currQuaternion = [currOrientation.w, currOrientation.x, currOrientation.y, currOrientation.z]
        # Convert the current oriention to euler from quaternion
        (currRoll, currPitch, currYaw) = euler_from_quaternion(currQuaternion)

        msgOrientation = msgPose.orientation
        msgQuaternion = [msgOrientation.w, msgOrientation.x, msgOrientation.y, msgOrientation.z]
        # Convert the message's oriention to euler from quaternion
        (msgRoll, msgPitch, msgYaw) = euler_from_quaternion(msgQuaternion)

        # Compare the Euler values
        # Roll
        covIndex = orientationCovarianceIndeces["Roll"]
        currRollCov = currCovariance[covIndex]
        msgRollCov = msgCovariance[covIndex]
        newRoll, newRollCov = self.compareValues(currRoll, msgRoll, currRollCov, msgRollCov)
        # Set new covariance value
        newCovariance[covIndex] = newRollCov

        # Pitch 
        covIndex = orientationCovarianceIndeces["Pitch"]
        currPitchCov = currCovariance[covIndex]
        msgPitchCov = currCovariance[covIndex]
        newPitch, newPitchCov = self.compareValues(currPitch, msgPitch, currPitchCov, msgPitchCov)
        # Set new covariance Value
        newCovariance[covIndex] = newPitchCov
        
        # Yaw
        covIndex = orientationCovarianceIndeces["Yaw"]
        currYawCov = currCovariance[covIndex]
        msgYawCov = msgCovariance[covIndex]
        newYaw, newYawCov = self.compareValues(currYaw, msgYaw, currYawCov, msgYawCov)
        # Set new covariance Value
        newCovariance[covIndex] = newYawCov

        # Convert to quaternion and set new orientation values

        (newQuatW, newQuatX, newQuatY, newQuatZ) = quaternion_from_euler(newRoll, newPitch, newYaw)

        newPose.orientation.w = newQuatW
        newPose.orientation.x = newQuatX
        newPose.orientation.y = newQuatY
        newPose.orientation.z = newQuatZ

        # Set the new pose and covariance as our new current values

        self.pose = newPose
        self.covariance = newCovariance
        
    # Compiles our representation into an object that we want 
    # return: PoseWithCovarianceStamped message (http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
    def getPoseWithCovarianceStamped(self):
        obj = PoseWithCovarianceStamped()
        obj.pose.pose = self.pose
        obj.pose.covariance = self.covariance
        obj.header.stamp = rospy.get_rostime() # Time Stamp
        return obj
