#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

joints_dict = {}


def joint_states_callback(message):

    joint_commands = JointState()

    joint_commands.header = message.header
    print("")
    for i, name in enumerate(message.name):

        # Storing arm joint names and positions
        joints_dict[name] = message.position[i]
        print("name: ", name)
        if name == "joint1":

            joints_dict["joint2"] = 0.0

    joint_commands.name = joints_dict.keys()
    joint_commands.position = joints_dict.values()
    print(joint_commands)

    # Publishing combined message containing all arm and finger joints
    pub.publish(joint_commands)

    return


if __name__ == "__main__":
    rospy.init_node("combined_joints_publisher")
    pub = rospy.Publisher("/joint_command", JointState, queue_size=1)
    rospy.Subscriber("/joint_command_desired", JointState, joint_states_callback, queue_size=1)
    rospy.spin()
