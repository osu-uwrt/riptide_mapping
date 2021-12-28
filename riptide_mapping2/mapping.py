#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default # can replace this with others
from rcl_interfaces.msg import SetParametersResult
import tf2_ros
from tf2_ros import TransformException
import transforms3d
import numpy as np
import copy
from vision_msgs.msg import Detection3DArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from riptide_mapping2.Estimate import Estimate
from math import pi

# Our overall data representation; each object has related information 
# We fill the publishers in __init__
objects = {
    "cutie": { # Old game object used for testing
        "pose": None, 
        "publisher" : None
    },
    "tommy": { # Tommygun game object.
        "pose":None,
        "publisher": None
    },
    "gman": { # GMan game object.
        "pose":None,
        "publisher": None
    },
    "bootlegger": { # Bootlegger game object.
        "pose":None,
        "publisher": None
    },
    "badge": { # Badge game object.
        "pose":None,
        "publisher": None
    },
    "gate": {
        "pose": None,
        "publisher" : None
    },
}

# Used to translate between DOPE ids and names of objects
object_ids = {
    0 : "gate",
    1 : "cutie", 
    2 : "tommy", 
    3 : "gman", 
    4 : "bootlegger", 
    5 : "badge", 
}

class MappingNode(Node):

    def __init__(self):
        super().__init__('riptide_mapping2') 
        
        # class variables 
        self.tf_buffer = tf2_ros.buffer.Buffer()
        self.tl = tf2_ros.TransformListener(self.tf_buffer, self)
        self.worldFrame = "world"
        self.cameraFrame = "{}stereo/left_optical".format(self.get_namespace())
        self.tf_brod = tf2_ros.transform_broadcaster.TransformBroadcaster(self)
        self.config = {}

        # declare the configuration data
        self.declare_parameters(
            namespace='',
            parameters=[
                ('stdev_cutoff', 1.0),
                ('angle_cutoff', 1.0),
                ('cov_limit', 1.0),
                ('k_value', 0.1),
                ('distance_limit', 10.0)
            ])
        
        # declare the fields for all of the models in the dict
        for object in object_ids.values():
            self.declare_parameters(
                namespace='',
                parameters=[
                    ('init_data.{}.pose.x'.format(object), 0.0),
                    ('init_data.{}.pose.y'.format(object), 0.0),
                    ('init_data.{}.pose.z'.format(object), 0.0),
                    ('init_data.{}.pose.yaw'.format(object), 0.0),
                    ('init_data.{}.covar.x'.format(object), 1.0),
                    ('init_data.{}.covar.y'.format(object), 1.0),
                    ('init_data.{}.covar.z'.format(object), 1.0),
                    ('init_data.{}.covar.yaw'.format(object), 1.0)
                ])

        # Creating publishers
        for field in objects:
            objects[field]["publisher"] = self.create_publisher(PoseWithCovarianceStamped, "{}mapping/{}".format(self.get_namespace(), field), qos_profile_system_default)

        # new parameter reconfigure call
        self.add_on_set_parameters_callback(self.paramUpdateCallback)

        # Subscribers
        self.create_subscription(Detection3DArray, "{}dope/detected_objects".format(self.get_namespace()), self.dopeCallback, qos_profile_system_default) # DOPE's information 

        # force the intial estimate to be published
        self.timer = self.create_timer(0.5, self.pubEstim)

    def pubEstim(self):
        for object_name in objects: 
            if not objects[object_name]["pose"] is None:
                objects[object_name]["publisher"].publish(objects[object_name]["pose"].get_pose_with_covariance_stamped())
        # stop the timer after the first tick
        #self.timer.cancel()

    # Handles reconfiguration for the mapping system.
    # NOTE: Reconfig reconfigures all values, not just the one specified in rqt.
    def paramUpdateCallback(self, params):

        for param in params:
            print(param.name, param.value)
            self.config[param.name, param.value]  

            # Get pose data from reconfig and update our map accordingly
            object_position = [
                self.config['init_data.{}.pose.x'.format(object)],
                self.config['init_data.{}.pose.y'.format(object)],
                self.config['init_data.{}.pose.z'.format(object)]
                ]
            object_yaw = self.config['init_data.{}.pose.yaw'.format(object)]
            object_covariance = [
                self.config['init_data.{}.covar.x'.format(object)],
                self.config['init_data.{}.covar.y'.format(object)],
                self.config['init_data.{}.covar.z'.format(object)],
                self.config['init_data.{}.covar.yaw'.format(object)]
                ]
            objects[object]["pose"] = Estimate(object_position, object_yaw, object_covariance)

            # Update filter 
            objects[object]["pose"].stdev_cutoff = self.config['stdev_cutoff']
            objects[object]["pose"].angle_cutoff = self.config['angle_cutoff']
            objects[object]["pose"].cov_limit = self.config['cov_limit']
            objects[object]["pose"].k_value = self.config['k_value']
            objects[object]["pose"].distance_limit = self.config['distance_limit']
            
            # Publish reconfigured data
            objects[object]["publisher"].publish(objects[object]["pose"].get_pose_with_covariance_stamped())
        
        return SetParametersResult(successful=True)


    # Handles merging DOPE's output into our representation
    # msg: Detection3DArray (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3DArray.html)
    def dopeCallback(self, msg):    
        # Context: This loop will run <number of different objects DOPE thinks it sees on screen> times
        # `detection` is of type Detection3D (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/Detection3D.html)
        for detection in msg.detections:

            now = rclpy.time.Time()
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.cameraFrame,
                    self.worldFrame,
                    now)
            except TransformException as ex:
                self.get_logger().error(f'Could not transform {self.cameraFrame} to {self.worldFrame}: {ex}')
                return

            # Note that we don't change the first loop to `detection in msg.detections.results` because we want the timestamp from the Detection3D object
            # Context: This loop will run <number of objects DOPE can identify> times 
            # `result` is of type ObjectHypothesisWithPose (http://docs.ros.org/en/lunar/api/vision_msgs/html/msg/ObjectHypothesisWithPose.html)
            for result in detection.results: 
                # Translate the ID that DOPE gives us to a name meaningful to us
                name = object_ids[result.id]

                # DOPE's frame is the same as the camera frame, specifically the left lens of the camera.
                # We need to convert that to the world frame, which is what is used in our mapping system 
                # Tutorial on how this works @ http://wiki.ros.org/tf/TfUsingPython#TransformerROS_and_TransformListener
                # Transform the pose part 
                p1 = PoseStamped()
                p1.header.frame_id = self.cameraFrame
                p1.header.stamp = now
                p1.pose = result.pose.pose

                convertedPos = self.tf_buffer.transform(p1, self.worldFrame)

                # Get the reading in the world frame message all together
                reading_world_frame = PoseWithCovarianceStamped()
                reading_world_frame.header.stamp = now
                reading_world_frame.header.frame_id = self.worldFrame
                reading_world_frame.pose.pose = copy.deepcopy(convertedPos.pose)

                # We do some error/reasonability checking with this
                reading_camera_frame = PoseWithCovarianceStamped()
                reading_camera_frame.header.frame_id = self.cameraFrame
                reading_camera_frame.header.stamp = now 
                reading_camera_frame.pose = result.pose

                # Merge the given position into our position for that object
                objects[name]["pose"].addPositionEstimate(reading_world_frame, reading_camera_frame, result.score)

                # Publish that object's data out 
                output_pose = objects[name]["pose"].get_pose_with_covariance_stamped()
                objects[name]["publisher"].publish(output_pose)

                # Publish /tf data for the given object 
                pose = output_pose.pose.pose # Get the embedded geometry_msgs/Pose (we don't need timestamp/covariance)
                translation = (pose.position.x, pose.position.y, pose.position.z) # Needs to be a 3-tuple rather than an object
                rotation = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w) # Needs to be a 4-tuple rather than an object
                time = now
                child = name + "_frame"
                parent = "world"
                tf.TransformBroadcaster().sendTransform(translation, rotation, time, child, parent)

    # Load the object's information from data
    def initial_object_pose(data):
        object_position = data["position"]
        object_yaw = data["yaw"]
        object_covariance = data["covariance"]
        object_covariance[3] *= pi / 180 # file uses degrees to be more human-readable, code uses rads
        return Estimate(object_position, object_yaw, object_covariance)


    # Handles the base object variance for each object.
    def base_object_variance(object_name, data):
        target_data = data[object_name]
        variance = np.array([target_data['x'],target_data['y'],target_data['z'],target_data['yaw']], float)
        
        return variance

def main(args=None):
    rclpy.init(args=args)

    node = MappingNode()

    rclpy.spin(node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()