from numpy.lib.function_base import cov
from numpy.lib.npyio import savez_compressed
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import Time
from math import sqrt, pi, atan2
from copy import deepcopy 
import math
import rospy 
import numpy as np

DEG_TO_RAD = pi / 180
ORIGIN_DEVIATION_LIMIT = 500

# Custom pose class that has commonly used mapping functionality 
class Estimate:
    
    def __init__(self, pos, yaw, cov):
        self.pos = pos # Length 3 List; x/y/z
        self.yaw = yaw # units: Degrees
        self.base_variance = np.array([0,0,0,0], float)
        self.covariance = cov # Length 4 List; x/y/z/yaw
        self.stamp = rospy.Time() # stamp/time

        self.stdev_cutoff = 1 # number of standard deviations
        self.angle_cutoff = 5 # units: degrees
        self.cov_limit = 0.01 # covariance value units: m^2
        self.k_value = 32768.0 # Used to calculate cov_multiplier. Determines how quickly the system converges on a value.
        self.distance_limit = 100 # units: meters
        
    # Takes in a reading and returns False if it's an invalid detection we want to filter out, True otherwise
    # msg: PoseWithCovarianceStamped representing robot in WORLD frame 
    # msg_camera_frame: PoseWithCovariancestamped representing robot in CAMERA frame 
    def isValidDetection(self, msg, msg_camera_frame, confidence):

        # Used throughout
        msg_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        msg_roll, msg_pitch, msg_yaw = euler_from_quaternion(msg_quat)
        self_quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        _, _, self_yaw = euler_from_quaternion(self_quat)

         # Reject detections that are not confident enough to be considered.
        if confidence < .5:
            print("Rejecting due to low confidence ({}).".format(confidence))
            return False
            
        # Reject detections that are too far from the origin (i.e. outside transdec)
        if msg.pose.pose.position.x >= ORIGIN_DEVIATION_LIMIT or msg.pose.pose.position.x <= -ORIGIN_DEVIATION_LIMIT or \
            msg.pose.pose.position.y >= ORIGIN_DEVIATION_LIMIT or msg.pose.pose.position.y <= -ORIGIN_DEVIATION_LIMIT or \
            msg.pose.pose.position.z >= ORIGIN_DEVIATION_LIMIT or msg.pose.pose.position.z <= -ORIGIN_DEVIATION_LIMIT:
            rospy.loginfo("Rejecting due to being too far from the origin.")
            return False
        
        # Reject detections that are too far away from the systems current estimate.
        if abs(msg.pose.pose.position.z - self.pos[2]) >= sqrt(self.covariance[2]) * self.stdev_cutoff:
             rospy.logwarn("Rejecting due to being unlikely in z-direction.")
             return False
        if abs(msg.pose.pose.position.x - self.pos[0]) >= sqrt(self.covariance[0]) * self.stdev_cutoff:
            rospy.loginfo("Rejecting due to being unlikely (x-direction)")
            return False
        if abs(msg.pose.pose.position.y - self.pos[1]) >= sqrt(self.covariance[1]) * self.stdev_cutoff:
            rospy.loginfo("Rejecting due to being unlikely (y-direction)")
            return False 

        # Yaw
        if ((msg_yaw > (self.yaw + self.angle_cutoff)) or (msg_yaw < (self.yaw - self.angle_cutoff))):
            return False

       


    

        # Reject detections that are unreasonably far away from the camera
        camera_x = msg_camera_frame.pose.pose.position.x
        camera_y = msg_camera_frame.pose.pose.position.y
        camera_z = msg_camera_frame.pose.pose.position.z
        distance_from_robot = sqrt((camera_x**2.0) + (camera_y**2.0) + (camera_z**2.0))
        if(distance_from_robot > self.distance_limit):
            return False
        
        # If the detection didn't fail any checks, assume it's a valid detection.
        return True 

    # Reconciles two estimates, each with a given estimated value and covariance
    # From https://ccrma.stanford.edu/~jos/sasp/Product_Two_Gaussian_PDFs.html,
    # Where estimate #1 is our current estimate and estimate #2 is the reading we just got in. 
    def update_value(self, val1, val2, cov1, cov2):
        new_mean = (val1 * cov2 + val2 * cov1) / (cov1 + cov2)
        new_cov = (cov1 * cov2) / (cov1 + cov2)

        # Ensure that covariance does not drop below the covariance threshold.
        if(new_cov < self.cov_limit):
            new_cov = self.cov_limit
        return new_mean, new_cov

    # Takes in a new DOPE reading and updates our estimate
    # msg: PoseWithCovarianceStamped of our new reading (WORLD FRAME)
    # msg_camera_frame: PoseWithCovarianceStamped of our new reading (CAMERA FRAME); just used for error checking 
    # Confidence: [0, 1] score of how confident DOPE is that it is the object in question
    def addPositionEstimate(self, msg, msg_camera_frame, confidence_score):

        # Debug 
        rospy.loginfo_throttle(2, "Position (x,y,z,yaw): ({}m, {}m, {}m, {}rad)" \
            .format(self.pos[0], self.pos[1], self.pos[2], self.yaw))
        rospy.loginfo_throttle(2, "Covariances (x,y,z,yaw): ({}m^2, {}m^2, {}m^2, {}rad^2)" \
            .format(self.covariance[0], self.covariance[1], self.covariance[2], self.covariance[3]))

        # Filter out "false positives"
        # If we make it past this, we assume it's a true positive and is actually the object
        if not self.isValidDetection(msg, msg_camera_frame, confidence_score):
            return

        # Update timestamp so it's the most recent routine 
        self.stamp = msg.header.stamp

        # Convenient aliases
        msg_pose = msg.pose.pose

        # Calculate covariance
        # Very simple conversion from DOPE score to covariance
        cov_multiplier = 32768.0 * math.log(-confidence_score + 2) # The lower the covariance, the more sure the system is. Basically just invert the score value.
        object_covaraince = self.base_variance * cov_multiplier # Multiply the base variance by the covariance multiplier we just calculated.
        
        # Update translational axes 
        # Note that saved covariance is 4-Long List representing x/y/z/yaw whereas message covariance is a full 36-long array that we take the diagonal of 
        self.pos[0], self.covariance[0] = self.update_value(self.pos[0], msg_pose.position.x, self.covariance[0], object_covaraince[0])
        self.pos[1], self.covariance[1] = self.update_value(self.pos[1], msg_pose.position.y, self.covariance[1], object_covaraince[1])
        self.pos[2], self.covariance[2] = self.update_value(self.pos[2], msg_pose.position.z, self.covariance[2], object_covaraince[2])
        
        # Yaw requires Quaterion->Euler transform, then we constrain it then feed it into our system 
        _, _, msg_yaw = euler_from_quaternion([msg_pose.orientation.x, msg_pose.orientation.y, msg_pose.orientation.z, msg_pose.orientation.w])
        msg_yaw = self.constrain_angle(self.yaw, msg_yaw)# + (math.pi/2)
        self.yaw, self.covariance[3] = self.update_value(self.yaw, msg_yaw, self.covariance[3], object_covaraince[3])
        

    # When getting distance between angles, situations like angle1=20 and angle2=340 will return that there's 320deg between them, not 40. 
    # This essentially constraints that!
    def constrain_angle(self, angle1, angle2):
        while angle2 > angle1 + pi:
            angle2 -= 2*pi
        while angle2 < angle1 - pi:
            angle2 += 2*pi
        return angle2
        
    # Compiles our representation into an object that we want 
    # return: PoseWithCovarianceStamped message (http://docs.ros.org/en/melodic/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html)
    def get_pose_with_covariance_stamped(self):

        # Stamp 
        output = PoseWithCovarianceStamped()
        output.header.frame_id = "world"
        output.header.stamp = self.stamp

        # Orientation
        output.pose.pose.orientation = Quaternion(*quaternion_from_euler(0, 0, self.yaw))
        output.pose.pose.position = Point(*self.pos)

        # Covariance 
        covariance = np.zeros((6, 6))
        covariance[0, 0] = self.covariance[0]
        covariance[1, 1] = self.covariance[1]
        covariance[2, 2] = self.covariance[2]
        covariance[3, 3] = self.covariance[3]
        output.pose.covariance = covariance.flatten()

        return output