#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates, LinkState, ModelState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from geometry_msgs.msg import Pose
from ur3_driver.msg import gripper_input
from std_msgs.msg import Bool
import argparse

class Grip:
    def __init__(self, link="robot") -> None:
        self.link_name = link
        self.robot_name = "jackal::base_link"

        self.robot  = Pose()

        self.sub = rospy.Subscriber("/gazebo/link_states", LinkStates, self.update_links, queue_size=10)
        self.pub = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)

    def update_links(self, msg : LinkStates):
        l = len(msg.name)
        i = 0


        for i in range(l):
            name = msg.name[i]
            print(name, self.robot_name)

            if name == self.robot_name:
                self.robot = msg.pose[i]
                self.robot : Pose
                print(self.robot)

                self.robot.position.z += 0.188069
                break

    def run(self):
        rate = rospy.Rate(1000)

        while not rospy.is_shutdown():
            out = SetModelStateRequest()
            outT = ModelState()

            out.model_state = outT

            outT.model_name = self.link_name
            outT.pose = self.robot
            # print(self.robot)
            outT.reference_frame = 'world'

            self.pub.call(out)
            
            rate.sleep()

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Please specify the link you want connect to the end effector')
    parser.add_argument('--link', type=str, default='robot')
    args = parser.parse_args()

    rospy.init_node("attach_ur3", anonymous=True)

    a = Grip(args.link)
    a.run()
