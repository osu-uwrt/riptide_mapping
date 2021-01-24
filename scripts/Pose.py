from geometry_msgs.msg import PoseWithCovarianceStamped

# Custom pose class that has commonly used mapping functionality 
class Pose:

    def __init__(self):
        self.pose = (0, 0, 0, 0, 0, 0) # xyzrpy
        self.size = (0, 0, 0) # xyz

    # TODO: This currently just updates it to the message. Implement something that gives more precedence to newer messages.
    # Takes in a new DOPE reading and updates our estimate
    def addPositionEstimate(self, msg):
        pass 

    def getPoseWithCovarianceStamped(self):
        pass