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
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, TransformStamped, Vector3
from riptide_mapping2.Estimate import Estimate
from math import pi
from tf_transformations import quaternion_inverse

DEG_TO_RAD = (pi/180)

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
        self.cameraFrame = "{}/stereo/left_optical".format(self.get_namespace())
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
                ('distance_limit', 10.0),
                ('confidence_cutoff', .7)
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

        # new parameter reconfigure call
        self.add_on_set_parameters_callback(self.paramUpdateCallback)

        # Creating publishers
        for field in objects:
            objects[field]["publisher"] = self.create_publisher(PoseWithCovarianceStamped, "mapping/{}".format(field), qos_profile_system_default)

        # Subscribers
        self.create_subscription(Detection3DArray, "dope/detected_objects", self.dopeCallback, qos_profile_system_default) # DOPE's information 

        # Timers
        self.publishTimer = self.create_timer(0.5, self.pubEstim) # publish the inital estimate
        self.paramUpdateTimer = self.create_timer(1.0, self.processParamUpdates) # propagate param updates
        self.paramUpdateTimer.cancel()

        # manually trigger the callback to load init params
        self.paramUpdateCallback(self.get_parameters(self._parameters.keys()))


    def pubEstim(self):
        for objectName in objects: 
            if not objects[objectName]["pose"] is None:
                # Publish that object's data out 
                output_pose = objects[objectName]["pose"].get_pose_with_covariance_stamped()
                objects[objectName]["publisher"].publish(output_pose)

                # Publish /tf data for the given object 
                newTf = TransformStamped()
                newTf.transform.translation = Vector3(x=output_pose.pose.pose.position.x, 
                    y=output_pose.pose.pose.position.y, z=output_pose.pose.pose.position.z)
                newTf.transform.rotation = output_pose.pose.pose.orientation
                newTf.header.stamp = output_pose.header.stamp
                newTf.child_frame_id = objectName + "_frame"
                newTf.header.frame_id = "world"
                self.tf_brod.sendTransform(newTf)

    # This timer will fire 1 second after paramter updates
    # This lets the parameter system reload all changes before modifying the estimates that need to be updated
    def processParamUpdates(self):
        success = True
        for objectName in objects: 
            if('init_data.{}.needs_update'.format(objectName) in self.config 
                and self.config['init_data.{}.needs_update'.format(objectName)]):
                try:
                    self.get_logger().info("Resetting estimate for {}".format(objectName))
                    # reset the update flag
                    self.config['init_data.{}.needs_update'.format(objectName)] = False

                    # Get pose data from reconfig and update our map accordingly
                    object_position = [
                        self.config['init_data.{}.pose.x'.format(objectName)],
                        self.config['init_data.{}.pose.y'.format(objectName)],
                        self.config['init_data.{}.pose.z'.format(objectName)]
                    ]
                    object_yaw = self.config['init_data.{}.pose.yaw'.format(objectName)] * DEG_TO_RAD # Need to convert this from degrees to radians.
                    object_covariance = [
                        self.config['init_data.{}.covar.x'.format(objectName)],
                        self.config['init_data.{}.covar.y'.format(objectName)],
                        self.config['init_data.{}.covar.z'.format(objectName)],
                        self.config['init_data.{}.covar.yaw'.format(objectName)]
                    ]

                    # Create a new Estimate object on reconfig.
                    objects[objectName]["pose"] = Estimate(object_position, object_yaw, object_covariance)

                    # Update filter 
                    objects[objectName]["pose"].stdev_cutoff = self.config['stdev_cutoff']
                    objects[objectName]["pose"].angle_cutoff = self.config['angle_cutoff'] * DEG_TO_RAD # Need to convert this from degrees to radians.
                    objects[objectName]["pose"].cov_limit = self.config['cov_limit']
                    objects[objectName]["pose"].k_value = self.config['k_value']
                    objects[objectName]["pose"].distance_limit = self.config['distance_limit']
                    objects[objectName]["pose"].confidence_cutoff = self.config['confidence_cutoff']
                    

                except Exception as e:
                    eStr = "Exception: {}, Exception message: {}".format(type(e).__name__, e)
                    self.get_logger().error("Error while updating object estimate for {}. {}".format(objectName, eStr))
                    success = False

        # if we have sucessfully updated all params, cancel the timer
        if(success):
            self.paramUpdateTimer.cancel()
        


    # Handles reconfiguration for the mapping system.
    # NOTE: Reconfig reconfigures all values, not just the one specified in rqt.
    def paramUpdateCallback(self, params):
        # cancel the propagate timer
        self.paramUpdateTimer.cancel()

        # update config and mark for re-estimation
        for param in params:
            print(param.name, param.value)
            self.config[param.name] = param.value
            for objectName in objects: 
                if(objectName in param.name):
                    self.config['init_data.{}.needs_update'.format(objectName)] = True

        # allow the timer to run
        self.paramUpdateTimer.reset()

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

                #Invert quaternion so x-axis faces camera
                p1.pose.orientation = quaternion_inverse(p1.pose.orientation)

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