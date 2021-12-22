import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Wrench, WrenchStamped, Vector3

import numpy as np


pub = rospy.Publisher('fake_wrench', WrenchStamped, queue_size=10)
rospy.init_node('test_wrench_publisher')
r = rospy.Rate(60)

random_force = np.random.rand(3,)
random_torque = np.random.rand(3,) * 0.1

wrench = WrenchStamped()
wrench.header.frame_id = "map"
wrench.wrench.force = Vector3(*random_force.tolist())
wrench.wrench.torque = Vector3(*random_force.tolist())

while not rospy.is_shutdown():
   pub.publish(wrench)
   r.sleep()
