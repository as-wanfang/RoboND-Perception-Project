#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_msgs.msg import Image
from sensor_msgs import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml
import time

import message_filters
from gqcnn.srv import GQCNNGraspPlanner
from gqcnn.msg import BoundingBox

#height: 576
#width: 1024
#distortion_model: plumb_bob
#K: [549.5941880572811, 0.0, 512.5, 0.0, 549.5941880572811, 288.5, 0.0, 0.0, 1.0]

# Callback function for your Point Cloud Subscriber
def callback(depth, rgb, camera_info):

    bounding_box = BoundingBox()
    bounding_box.minX = 100
    bounding_box.minY = 50
    bounding_box.maxX = 924
    bounding_box.maxY = 526

    rospy.wait_for_service('plan_gqcnn_grasp')

    try:
        plan_routine = rospy.ServiceProxy('plan_gqcnn_grasp', GQCNNGraspPlanner)

        # TODO: Insert your message variables to be sent as a service request
        resp = plan_routine(rgb, depth, camera_info, bounding_box)

        print ("Response: ",resp.success)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    # try:
    #     pr2_mover(cloud_objects, cloud_table, cluster_indices, detected_objects_list)
    # except rospy.ROSInterruptException:
    #     pass

# function to load parameters and request PickPlace service
def pr2_mover(cloud_objects, cloud_table, cluster_indices, object_list):

    # TODO: Get/Read parameters
    object_list_param = rospy.get_param('/object_list')
    dropbox_param = rospy.get_param('/dropbox')

    # TODO: Parse parameters into individual variables
    test_scene_num = Int32()
    test_scene_num.data = 1

    dropbox_left = dropbox_param[0]['position']
    dropbox_right = dropbox_param[1]['position']

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # TODO: Loop through the pick list
    grasp_list = rospy.get_param('/grasp_list')[0]
    for i in range(len(object_list)):
        object = object_list[i]
        collision_indices = []
        for index in range(cloud_objects.size):
            if index not in cluster_indices[i]:
                collision_indices.append(index)
        cloud_collision = cloud_objects.extract(cluster_indices[i])
        print "cloud_objects size: %s"%cloud_objects.size
        print "cloud_collision size: %s"%cloud_collision.size
        collison_pub.publish(pcl_to_ros(cloud_collision))
        time.sleep(3)

        object_name = String()
        object_name.data = object.label

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        # TODO: Create 'place_pose' for the object
        pick_pose = Pose()
        pick_pose.position.x = grasp_list[object.label]['position']['x']
        pick_pose.position.y = grasp_list[object.label]['position']['y']
        pick_pose.position.z = grasp_list[object.label]['position']['z']

        place_pose = Pose()

        # TODO: Assign the arm to be used for pick_place
        # specify the place_pose here, vary the x value of position with [-0.2,0.2] so that objects will not stack in the dropbox
        arm_name = String()
        if True:
            arm_name.data = 'left'
            place_pose.position.x = dropbox_left[0] - 0.2
            place_pose.position.y = dropbox_left[1]
            place_pose.position.z = 0.9
        else:
            arm_name.data = 'right'
            place_pose.position.x = dropbox_right[0] - 0.2 + i*0.05
            place_pose.position.y = dropbox_right[1]
            place_pose.position.z = 0.9


        # collision

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        try:
            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)

            # TODO: Insert your message variables to be sent as a service request
            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)

            print ("Response: ",resp.success)

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

if __name__ == '__main__':

    # TODO: ROS node initialization
    rospy.init_node('perception', anonymous=True)

    # TODO: Create Subscribers
    depth_sub = message_filters.Subscriber("/camera/depth_registered/image_raw", Image)
    rgb_sub = message_filters.Subscriber("/camera/rgb/image_raw", Image)
    info_sub = message_filters.Subscriber('/camera/rgb/camera_info', CameraInfo)

    ts = message_filters.TimeSynchronizer([depth_sub, rgb_sub, info_sub], 10)
    ts.registerCallback(callback)

    # TODO: Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()
