#!/usr/bin/env python3

from cv2 import erode
import rospy
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import numpy as np
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Bool, Int8

waypoints = [
    [2.27, -2.46, 0],
    [8.28, -3.923, 0]
]


class RobotMover:
    def __init__(self) -> None:
        self.position_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.position_callback)
        self.vel_pub = rospy.Publisher("/jackal/jackal_velocity_controller/cmd_vel", Twist, queue_size=1)
        self.dep_sub = rospy.Subscriber("/depositied", Bool, self.depositied_callaback,queue_size=1)

        self.can_nums_sub = rospy.Subscriber("/can_nums", Int8, self.num_cans_callback,queue_size=1)

        self.cmd_vel = Twist()

        self.robot_data = [0,0,0,0]

        self.received_first = False
        self.depositied = False

        self.has_seen = False
        self.num_cans = 0

        self.wait_for_pose()

        print("recieved first")

    def num_cans_callback(self, msg: Int8):
        self.num_cans = msg.data
        self.has_seen = True

    def depositied_callaback(self, msg: Bool):
        self.depositied = msg.data

    def wait_for_pose(self):
        rate = rospy.Rate(60)

        while not rospy.is_shutdown() and not self.received_first:
            rate.sleep()

    def set_speeds(self, lin=0, ang=0):
        self.cmd_vel.linear.x = lin
        self.cmd_vel.angular.z = ang


    def position_callback(self, msg:ModelStates):
        id = -1
        
        for i, name in enumerate(msg.name):
            if name == "jackal":
                id = i
        if id == -1:
            return

        pose = msg.pose[id]

        p_x, p_y, p_z = pose.position.x,pose.position.y,pose.position.z
        o_x, o_y, o_z, o_w = pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w

        roll, pitch, yaw = euler_from_quaternion([o_x, o_y, o_z, o_w])

        # print(np.degrees(roll), np.degrees(pitch), np.degrees(yaw))

        self.robot_data = [p_x,p_y, p_z, yaw]

        self.received_first = True

    def move_to(self, end_pose):
        x_t, y_t, z_t, yaw_t = end_pose

        target_angle = np.arctan2(y_t - self.robot_data[1], x_t - self.robot_data[0])

    def angle_error(self, target_pose):
        error_ang = np.arctan2((target_pose[1] - self.robot_data[1]), target_pose[0] - self.robot_data[0]) - self.robot_data[3]
        
        if abs(error_ang) > np.pi:
            error_ang = np.sign(error_ang) * abs(error_ang) % np.pi

        return error_ang



    def move_rotate_angle(self, target_pose, target_angle):
        rate = rospy.Rate(60)

        error_ang = self.angle_error(target_pose)

        while abs(error_ang) > np.radians(2):
            error_ang = self.angle_error(target_pose)

            ang_control = error_ang * 1

            print(error_ang)

            self.set_speeds(ang=ang_control)  

            self.pub()
            rate.sleep()

        error_dist = np.sqrt((self.robot_data[0] - target_pose[0])**2 +  (self.robot_data[1] - target_pose[1])**2 ) 

        while abs(error_ang) > np.radians(2) or abs(error_dist) > .1:
            error_ang = self.angle_error(target_pose)

            error_dist = np.sqrt((self.robot_data[0] - target_pose[0])**2 +  (self.robot_data[1] - target_pose[1])**2 ) 

            vel_control = min(error_dist * .25, 1)
            ang_control = error_ang * 1

            print(error_ang, error_dist)

            self.set_speeds(lin=vel_control,ang=ang_control)  

            self.pub()
            rate.sleep()

        error_ang = target_angle -  self.robot_data[3]

        if abs(error_ang) > np.pi:
            error_ang = np.sign(error_ang) * abs(error_ang) % np.pi

        self.set_speeds()

        while abs(error_ang) > np.radians(2):
            error_ang = target_angle -  self.robot_data[3]

            if abs(error_ang) > np.pi:
                error_ang = np.sign(error_ang) * abs(error_ang) % np.pi

            ang_control = error_ang * 1

            self.set_speeds(ang=ang_control)  

            self.pub()
            rate.sleep()
    
    def pub(self):
        self.vel_pub.publish(self.cmd_vel)


    def run(self):
        rate = rospy.Rate(60)

        while not self.has_seen and not rospy.is_shutdown():
            rate.sleep()

        if rospy.is_shutdown():
            return

        print("Running",self.num_cans, "times!")

        for i in range(self.num_cans):
            self.move_rotate_angle(waypoints[1][:2], waypoints[1][2])

            while not self.depositied:
                if rospy.is_shutdown():
                    return
                
                rate.sleep()

            self.move_rotate_angle(waypoints[0][:2], waypoints[0][2])

        # while not rospy.is_shutdown():
        #     rate.sleep()


if __name__ == "__main__":
    rospy.init_node("robot_mover")

    p = RobotMover()

    p.run()