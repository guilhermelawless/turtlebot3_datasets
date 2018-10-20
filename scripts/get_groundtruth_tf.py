#!/usr/bin/env python2.7
import rospy
import rosbag
import sys
import tf as tfros

PARENT_FRAME = 'mocap'
CHILD_FRAME = 'mocap_laser_link'

def to_static_usage(tf):
    tr = tf.transform.translation
    rt = tf.transform.rotation
    roll, pitch, yaw = tfros.transformations.euler_from_quaternion( [rt.x, rt.y, rt.z, rt.w] )
    return 'rosrun tf2_ros static_transform_publisher {} {} {} {} {} {} {} {}'.format(
        tr.x, tr.y, tr.z, yaw, roll, pitch, tf.header.frame_id, 'FIXED_FRAME'
    )

argc = len(sys.argv)
if argc < 2:
    print('Usage: {} INPUT_BAG'.format(sys.argv[0]))
    sys.exit(1)

bag_file = sys.argv[1]

with rosbag.Bag( open(bag_file), mode='r', allow_unindexed=False ) as bag:
    for _, msg, _ in bag.read_messages(topics=['/tf']):
        for tf in msg.transforms:
            if tf.header.frame_id == PARENT_FRAME and tf.child_frame_id == CHILD_FRAME:
                print(tf)
                print('\n\n-----To run as static transform (change FIXED_FRAME to map/odom/other):------\n')
                print(to_static_usage(tf))
                sys.exit(0)

print('Could not find any transform between {} and {} in {}'.format(PARENT_FRAME, CHILD_FRAME, bag_file))
