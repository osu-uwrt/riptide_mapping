from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
import rospy 

# Custom pose class that has commonly used mapping functionality 
class CustomPose:

    def __init__(self):
        self.pose = None # Likely Pose Type
        self.size = None # Likely Vector3 Type

        # TODO: Implement Uncertainty 
        self.uncertainty = 0

    # TODO: This currently just updates it to the message. Implement something that gives more precedence to newer messages.
    # Takes in a new DOPE reading and updates our estimate
    # msg: Detection3D Object (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3D.html)
    def addPositionEstimate(self, msg):

        # TODO: If we don't have an estimate for this object yet, this is our new estimate!
        if (self.pose == None):
            pass

        # TODO: Otherwise, do a weighted update of our representation! Figure out how this should work. We want to give more weight to newer results, as uncertainty builds up over time.
        else:
            pass

    # Compiles our representation into an object that we want 
    # return: PoseWithCovarianceStamped message (http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
    def getPoseWithCovarianceStamped(self):
        obj = PoseWithCovarianceStamped()
        # TODO: Fill the object's pose with all the data we have access to (xyz/rpy/lwh should be enough)
        obj.header.stamp = rospy.get_rostime() # Time Stamp
        return obj