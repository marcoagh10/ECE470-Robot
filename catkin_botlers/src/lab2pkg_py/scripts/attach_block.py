#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates, LinkState
from geometry_msgs.msg import Pose
from ur3_driver.msg import gripper_input
from std_msgs.msg import Bool, String
import argparse

class Grip:
    def __init__(self, link="coke_can0::link") -> None:
        self.coke_name = link

        self.coke  = Pose()
        self.gripper = Pose()

        self._attach = False

        self.sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.update_links, queue_size=10)
        self.sub_gripper = rospy.Subscriber("/gripper/position", Pose, self.update_gripper, queue_size=10)
        self.pub = rospy.Publisher("/gazebo/set_link_state", LinkState, queue_size=5)
        self.gripper_com = rospy.Subscriber("/ur3/grip", Bool, self.attach_input, queue_size=1)
        self.target_com = rospy.Subscriber("/ur3/target", String, self.set_target, queue_size=1)

    def set_target(self, msg: String):
        self.coke_name = msg.data

    def update_gripper(self, msg: Pose):
        self.gripper = msg

    def attach_input(self, msg:Bool):
        self._attach = bool(msg.data)

    def update_links(self, msg : LinkStates):
        l = len(msg.name)
        i = 0

        for i in range(l):
            name = msg.name[i]

            if name == self.coke_name:
                i += 1
                self.coke = msg.pose[i]

            if i == 2:
                break

        # print("COKE", self.coke)

    def attach(self):
        self._attach = True

    def run(self):
        rate = rospy.Rate(1000)

        while not rospy.is_shutdown():
            if self._attach:
                out = LinkState()

                out.link_name = self.coke_name
                out.pose = self.gripper
                out.reference_frame = 'world'

                self.pub.publish(out)
            
            rate.sleep()

    def detach(self):
        self._attach = False

if __name__ == "__main__":

    # parser = argparse.ArgumentParser(description='Please specify the link you want connect to the end effector')
    # parser.add_argument('--link', type=str, default='coke_can::link', required=False)
    # args = parser.parse_args()

    rospy.init_node("attach", anonymous=True)

    a = Grip("coke_can0::link")
    a.run()
