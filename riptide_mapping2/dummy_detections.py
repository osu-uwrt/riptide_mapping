import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovariance
from vision_msgs.msg import Detection3DArray
from vision_msgs.msg import Detection3D
from vision_msgs.msg import ObjectHypothesisWithPose

MODELS_TOPIC = "/gazebo/model_states"
DETECTIONS_TOPIC = "/tempest/dope/detected_objects"


# Used to assign object ids to models. copied from mapping
object_ids = {
    0 : "gate",
    1 : "cutie", 
    2 : "tommy", 
    3 : "gman", 
    4 : "bootlegger", 
    5 : "badge", 
}

#ROS node to output dummy detections to ros topic
#uses the positions of the objects output by gazebo and publishes
#them in a Detection3DArray msg with some gaussian noise
class DummyDetections(Node):
    def __init__(self):
        super().__init__("dummy_detections")
        self.sub = self.create_subscription(ModelStates, MODELS_TOPIC, self.subCallback, qos_profile_system_default)
        self.pub = self.create_publisher(Detection3DArray, DETECTIONS_TOPIC, qos_profile_system_default)
        self.get_logger().info("Started dummy detection node.")

    
    def subCallback(self, msg: ModelStates):
        detectArray = Detection3DArray()
        
        #formulate header
        header = Header()
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()
        
        detectArray.header = header
        
        print("Received new msg with {} names, {} poses, and {} twists".format(len(msg.name), len(msg.pose), len(msg.twist)))
        
        for i in range(0, len(msg.name)): #msg.name is deceiving. it is an array containing the names of all models
            name: int = msg.name[i]
            pose: Pose = msg.pose[i]
            
            # #do not try to continue if the object is not in the dict of objects we care about
            # if not name in object_ids.values():
            #     continue
            
            # id = list(object_ids.keys())[list(object_ids.values()).index(name)]
            
            #populate detection. looks like mapping only uses results so I'll just populate that and also header because its easy
            detection = Detection3D()
            detection.header = header
            
            hypothesis = ObjectHypothesisWithPose()
            # hypothesis.id = id #TODO: figure out an alternative that makes galactic not complain
            # hypothesis.score = 0.75
            
            posewithcovariance = PoseWithCovariance()
            posewithcovariance.pose = pose
            #TODO: covariance
            
            hypothesis.pose = posewithcovariance
            
            detection.results.append(hypothesis)
            detectArray.detections.append(detection)
            
        print("publishing {} detections".format(len(detectArray.detections)))
        
        self.pub.publish(detectArray)
        
        


def main(args=None):
    rclpy.init(args=args)
    
    dummydetections = DummyDetections()
    rclpy.spin(dummydetections)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
