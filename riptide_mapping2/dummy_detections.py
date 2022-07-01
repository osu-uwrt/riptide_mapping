from __future__ import annotations

from math import pi

import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovariance
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from std_msgs.msg import Header
from transforms3d.euler import euler2quat, quat2euler
from vision_msgs.msg import (Detection3D, Detection3DArray,
                             ObjectHypothesisWithPose)

DETECTIONS_TOPIC = "/tempest/dope/detected_objects" #topic to publish "detections" to 
MAPPING_NODE_NAME = "/tempest/riptide_mapping2"
PARAMS_TIMER_INTERVAL = 1
DETECTION_TIMER_INTERVAL = 0.1 #timer interval in seconds
DETECTIONS_SCORE = 0.8         #score. not sure if mapping actually uses this lol
NOISE_SCALE = 0.1           #use this to increase or decrease the strength of the noise
TO_RAD = pi / 180.0

# Used to assign object ids to models. copied from mapping
objects : dict[str, PoseWithCovariance] = {
    "gate"       : None,
    "cutie"      : None,
    "tommy"      : None,
    "gman"       : None,
    "bootlegger" : None,
    "badge"      : None
}

#ROS node to output dummy detections to ros topic
#uses the positions of the objects output by gazebo and publishes
#them in a Detection3DArray msg with some noise
class DummyDetections(Node):
    def __init__(self):
        super().__init__("dummy_detections")
        self.pub = self.create_publisher(Detection3DArray, DETECTIONS_TOPIC, qos_profile_system_default)
        
        self.get_logger().info("Creating mapping parameter service...")
        self.paramsService = self.create_client(GetParameters, MAPPING_NODE_NAME + "/get_parameters")
        self.paramsService.wait_for_service()
        self.get_logger().info("...done")
                
        self.serviceTimer = self.create_timer(PARAMS_TIMER_INTERVAL, self.getMappingParams)
        self.publishTimer = self.create_timer(DETECTION_TIMER_INTERVAL, self.timerCallback)
        
        self.get_logger().info("Started dummy detection node.")
        
        
    def getMappingParams(self):
        paramsRequest = GetParameters.Request()
        paramsRequest.names = []
        
        #populate list of param names to retrieve.
        for objectname in objects:
            for metric in ["covar", "pose"]:
                for value in ["x", "y", "yaw", "z"]:
                    paramsRequest.names.append("init_data.{}.{}.{}".format(objectname, metric, value))
        
        future = self.paramsService.call_async(paramsRequest)
        future.add_done_callback(self.processMappingParams) #add callback because spinning until future complete will hang if node is added to multiple executors
        
    
    def processMappingParams(self, future: rclpy.Future):        
        result: GetParameters.Response = future.result()
        if len(result.values) == 0: #something was wonky
            self.get_logger().error("Failed to properly retreive mapping params! Check that all object names in the objects dict match the names in the config.yaml!")
        
        # set params. see getMappingParams for the order that params are in (its the three for loops)
        idx = 0
        for objectname in objects:
            objects[objectname] = PoseWithCovariance()
            
            #params are in order [covar.x, covar.y, covar.yaw, covar.z, pose.x, pose.y, pose.yaw, pose.z]
            objects[objectname].covariance[0]   = result.values[idx].double_value     #covar x
            objects[objectname].covariance[7]   = result.values[idx + 1].double_value #covar y
            objects[objectname].covariance[14]  = result.values[idx + 2].double_value #covar yaw 
            objects[objectname].covariance[35]  = result.values[idx + 3].double_value
            
            objects[objectname].pose.position.x = result.values[idx + 4].double_value
            objects[objectname].pose.position.y = result.values[idx + 5].double_value
            objects[objectname].pose.position.z = result.values[idx + 7].double_value
            
            yaw = result.values[idx + 6].double_value * TO_RAD
            quat = euler2quat(0, 0, yaw) #returns wxyz
            objects[objectname].pose.orientation.w = quat[0]
            objects[objectname].pose.orientation.x = quat[1]
            objects[objectname].pose.orientation.y = quat[2]
            objects[objectname].pose.orientation.z = quat[3]
                        
            idx += 8
        

    
    def timerCallback(self):
        detectArray = Detection3DArray()
        
        #formulate header
        header = Header()
        header.frame_id = "world"
        header.stamp = self.get_clock().now().to_msg()
        
        detectArray.header = header
                
        for object in objects: #msg.name is deceiving. it is an array containing the names of all models
            pose: PoseWithCovariance = objects[object]
            if pose is None:
                continue
            
            #generate noise
            [r, p, y] = quat2euler([pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z]) #wxyz
            noise = np.random.normal(0, NOISE_SCALE, 7)
            
            #add noise to stuff
            pose.pose.position.x += noise[0]
            pose.pose.position.y += noise[1]
            pose.pose.position.z += noise[2]
            r += noise[3]
            p += noise[4]
            y += noise[5]
                        
            #rpy to quat that boi
            newQuat = euler2quat(r, p, y) #returns in WXYZ order
            pose.pose.orientation.w = newQuat[0]
            pose.pose.orientation.x = newQuat[1]
            pose.pose.orientation.y = newQuat[2]
            pose.pose.orientation.z = newQuat[3]
            
            #populate detection. looks like mapping only uses results so I'll just populate that and also header because its easy
            detection = Detection3D()
            detection.header = header
            
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = object
            hypothesis.hypothesis.score = DETECTIONS_SCORE
            hypothesis.pose = pose
            
            detection.results.append(hypothesis)
            detectArray.detections.append(detection)
            
        self.pub.publish(detectArray)
        
        


def main(args=None):
    rclpy.init(args=args)
    
    dummydetections = DummyDetections()
    rclpy.spin(dummydetections)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
