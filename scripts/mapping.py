#!/usr/bin/env python

import rospy

# TODO: Define our representation. Something like this will probably work, where the six-tuple is xyzrpy
currentPositions = {
    "prop1": (0, 0, 0, 0, 0, 0), 
    "prop2": (0, 0, 0, 0, 0, 0), 
    "prop3": (0, 0, 0, 0, 0, 0), 
    "prop4": (0, 0, 0, 0, 0, 0)
}

# Handles merging DOPE's output into our representation
def dopeCallback(msg):

    # TODO: Merge the changes from DOPE into our representation
    # Will need to do a little math to figure out the best way to update it.
    # We don't want to straight-up overwrite it every time and ignore all our previous data points, but we also know new data is more accurate so we want to use it.

    # TODO: Publish every object's relevant points 

    # Can be deleted once we have actual content in this function 
    pass

# Kept as an example of what a callback that does some processing looks like, can be deleted once we get our feet under us 
def bboxCb(msg):
    objects = {}
    for b in msg.bounding_boxes:
        if not objects.has_key(b.Class) or b.probability > objects[b.Class].probability:
            objects[b.Class] = b
    msg.bounding_boxes = objects.values()
    bboxPub.publish(msg)

if __name__ == '__main__':

    rospy.init_node("mapping")

    # TODO: Read initial positions into whatever our representation is 

    # TODO: Register all our subscribers and publishers 
    # This is left over from the copy from riptide_vision, and is just an example of what pubs/subs look like.
    # We will need at least one subscriber to parse in DOPE's output, and at least one publisher to publish our current estimate for each object of interest. 
    rospy.Subscriber("command/camera", Int8, cameraSelectionCb)
    rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, bboxCb)
    cameraPub = rospy.Publisher("darknet_ros/input_image", Image, queue_size=1)
    bboxPub = rospy.Publisher("state/bboxes", BoundingBoxes, queue_size=1)
    cameraSub = rospy.Subscriber("stereo/left/image_rect_color", Image, cameraCb)

    rospy.spin()
