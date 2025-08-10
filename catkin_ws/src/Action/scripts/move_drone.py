#! /usr/bin/env python

import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


class DroneActionClient:
    def __init__(self):
        rospy.init_node('drone_action_client')
        self.action_server_name = '/ardrone_action_server'
        self.client = actionlib.SimpleActionClient(self.action_server_name, ArdroneAction)
        self.move_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.takeoff_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/drone/land', Empty, queue_size=1)
        self.move_msg = Twist()
        self.takeoff_msg = Empty()
        self.land_msg = Empty()
        self.rate = rospy.Rate(1)
        self.nImage = 1

    def wait_for_server(self):
        rospy.loginfo('Waiting for action server ' + self.action_server_name)
        self.client.wait_for_server()
        rospy.loginfo('Action server found... ' + self.action_server_name)

    def send_goal(self, nseconds):
        goal = ArdroneGoal()
        goal.nseconds = nseconds
        self.client.send_goal(goal, feedback_cb=self.feedback_callback)

    def feedback_callback(self, feedback):
        print('[Feedback] image n.%d received' % self.nImage)
        self.nImage += 1

    def takeoff(self, duration=3):
        rospy.loginfo('Taking off...')
        for _ in range(duration):
            self.takeoff_pub.publish(self.takeoff_msg)
            time.sleep(1)

    def land(self, duration=3):
        rospy.loginfo('Landing...')
        for _ in range(duration):
            self.move_msg.linear.x = 0
            self.move_msg.angular.z = 0
            self.move_pub.publish(self.move_msg)
            self.land_pub.publish(self.land_msg)
            time.sleep(1)

    def move_drone(self):
        self.move_msg.linear.x = 1
        self.move_msg.angular.z = 1
        self.move_pub.publish(self.move_msg)
        rospy.loginfo('Moving around...')

    def execute(self):
        self.wait_for_server()
        self.takeoff()
        self.send_goal(10)

        state_result = self.client.get_state()

        while state_result in [GoalStatus.PENDING, GoalStatus.ACTIVE, GoalStatus.PREEMPTING]:
            self.move_drone()
            self.rate.sleep()
            state_result = self.client.get_state()
            rospy.loginfo('Moving around...')
            rospy.loginfo("state_result: " + str(state_result))

        rospy.loginfo("[Result] State: " + str(state_result))
        if state_result == GoalStatus.ABORTED:
            rospy.logerr("Something went wrong in the Server Side")
        if state_result == GoalStatus.REJECTED:
            rospy.logwarn("The goal was rejected by the Server")
        if state_result == GoalStatus.SUCCEEDED:
            rospy.loginfo("Goal succeeded!")

        self.land()

def main():
    drone_client = DroneActionClient()
    drone_client.execute()

if __name__ == '__main__':
    main()