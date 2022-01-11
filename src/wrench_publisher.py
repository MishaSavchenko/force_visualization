import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Wrench, WrenchStamped, Vector3

import numpy as np


pub = rospy.Publisher('fake_wrench', WrenchStamped, queue_size=10)
rospy.init_node('test_wrench_publisher')

# r = rospy.Rate(0.5)
r = rospy.Rate(60)

def ft_data_to_wrench_stamped(force, torque,  frame_id = "map"):
    wrench = WrenchStamped()
    wrench.header.frame_id = frame_id
    wrench.wrench.force = Vector3(*force.tolist())
    wrench.wrench.torque = Vector3(*torque.tolist())
    return wrench

def get_random_ft_data(torque_modifier=0.1):
    random_force = (np.random.rand(3,) - 0.5) * 2
    random_torque = (np.random.rand(3,) - 0.5) * torque_modifier
    return (random_force, random_torque)


last_ft_data = get_random_ft_data()
while not rospy.is_shutdown():
    next_ft_data = get_random_ft_data()

    force_delta = next_ft_data[0] - last_ft_data[0]
    torque_delta = next_ft_data[1] - last_ft_data[1]
    for t in np.linspace(0, 1, 25):
        force = last_ft_data[0] + force_delta * t
        torque = last_ft_data[1] + torque_delta * t

        wrench = ft_data_to_wrench_stamped(force, torque)
        rospy.loginfo("\n%s",wrench)
        pub.publish(wrench)
        r.sleep()

    last_ft_data = next_ft_data
