from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
import rospy 

# Custom pose class that has commonly used mapping functionality 
class CustomPose:

    def __init__(self):
        self.pose = None # Pose Type
        self.covariance = None # float64[36] type 

    # Takes in a new DOPE reading and updates our estimate
    # msg: PoseWithCovarianceStamped Object (http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovariance.html)
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
    def addPositionEstimate(self, msg):
        pass 

    # Compiles our representation into an object that we want 
    # return: PoseWithCovarianceStamped message (http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
    def getPoseWithCovarianceStamped(self):
        obj = PoseWithCovarianceStamped()
        obj.pose.pose = self.pose
        obj.pose.covariance = self.covariance
        obj.header.stamp = rospy.get_rostime() # Time Stamp
        return obj