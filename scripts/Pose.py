from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
import rospy 

# Custom pose class that has commonly used mapping functionality 
class CustomPose:

    def __init__(self):
        self.pose = None # Pose Type
        self.covariance = None # float64[36] type 

    # TODO: This currently just updates it to the message. Implement something that gives more precedence to newer messages.
    # Takes in a new DOPE reading and updates our estimate
    # msg: Detection3D Object (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3D.html)
    def addPositionEstimate(self, msg):

        # TODO: Figure out how a weighted update should work, then implement it.
        # The general idea is to give more weight to newer results, as uncertainty builds up over time.
        # More weight should also be given to results where DOPE gives us a high confidence rating for it.
        # Past that, not really sure exactly how this should work. 
        
        pass 

    # Compiles our representation into an object that we want 
    # return: PoseWithCovarianceStamped message (http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
    # TODO: *Should* work, but needs tested!
    def getPoseWithCovarianceStamped(self):
        obj = PoseWithCovarianceStamped()
        obj.pose.pose = self.pose
        obj.pose.covariance = self.covariance
        obj.header.stamp = rospy.get_rostime() # Time Stamp
        return obj