#!/usr/bin/env python2.7
import rospy
import rosbag
import sys
import tf as tfros

'''
Use this file to fix a specific TF time stamp (did not synchronize time, only works if the time difference is constant (not a big assumption for a small datasets))

The result of this script is the same bag but with header stamps instead of the recorded time of receiving each msg.
For the 'mocap' TFs it will also subtract TIME_OFFSET to the msg stamp.

In case of slam_easy.bag , measured (1539703367.760256098 - 1539699406.298793935) = 3961.461462163 second offset between the robot TF and the mocap TF
'''

PARENT_FRAME = 'mocap'

argc = len(sys.argv)
if argc < 4:
    print('Usage: {} TIME_OFFSET INPUT_BAG OUTPUT_BAG'.format(sys.argv[0]))
    sys.exit(1)

offset = float(sys.argv[1])
in_bag_file = sys.argv[2]
out_bag_file = sys.argv[3]

print('Applying offset of {:f} to frames which parent is {}'.format(offset, PARENT_FRAME))

out_bag = rosbag.Bag( open(out_bag_file, 'wb'), mode='w', allow_unindexed=False )
with rosbag.Bag( open(in_bag_file), mode='r', allow_unindexed=False ) as in_bag:
    try:
        import progressbar
    except:
        print("To get a progress bar display:\npip install --user progressbar2")
        bar = None
    else:
        bar = progressbar.ProgressBar(max_value=in_bag.get_message_count(), redirect_stdout=True, end=' ')

    msg_counter = 0
    for topic, msg, t in in_bag.read_messages():
        msg_counter += 1
        if bar:
            bar.update(msg_counter)

        if topic == "/tf":
            for idx, t in enumerate(msg.transforms):
                if msg.transforms[idx].header.frame_id == PARENT_FRAME:
                    msg.transforms[idx].header.stamp -= rospy.Duration(offset)
            out_bag.write(topic, msg, msg.transforms[0].header.stamp)

        elif msg._has_header:
            out_bag.write(topic, msg, msg.header.stamp)
        else:
            out_bag.write(topic, msg, t)

out_bag.close()
