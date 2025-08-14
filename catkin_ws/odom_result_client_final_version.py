#!/usr/bin/env python
import rospy, actionlib
from ros_wall_following.msg import OdomRecordAction, OdomRecordGoal, OdomRecordFeedback

def feedback_cb(feedback):
    rospy.loginfo(f"[Feedback] Total distance: {feedback.current_total:.2f} m")


def main():
    rospy.init_node('odom_result_client')

    # Connect to the action server
    client = actionlib.SimpleActionClient('record_odom', OdomRecordAction)
    rospy.loginfo("Waiting for record_odom action server...")
    client.wait_for_server()

    # Send an empty goal to start recording
    rospy.loginfo("Sending goal to start odometry recording.")
    client.send_goal(OdomRecordGoal(), feedback_cb=feedback_cb)

    # Wait until the action is complete
    client.wait_for_result()
    result = client.get_result()

    rospy.loginfo("Final odometry list:")
    for idx, p in enumerate(result.list_of_odoms):
        rospy.loginfo(f"{idx}: x={p.x:.2f}, y={p.y:.2f}, theta={p.z:.2f}")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
