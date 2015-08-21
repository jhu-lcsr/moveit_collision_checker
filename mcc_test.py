#!/usr/bin/env python

import rospy
from moveit_collision_checker_py import Checker, init_roscpp

def main():
    rospy.init_node('mcc_test')
    init_roscpp();

    robot_description_param = rospy.get_param('~robot_description_param')
    root_link = rospy.get_param('~root_link')
    scene_topic = rospy.get_param('~scene_topic')

    mcc = Checker(robot_description_param, root_link, scene_topic)

    position = (0.5,0.5,0.25)
    orientation = (0.0, 0.0, 0.0, 1.0)
    joint_positions = 8 * [0.0]

    r = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        mcc.spin_once()
        valid = mcc.check_state(position, orientation, joint_positions)
        rospy.loginfo('Valid: {}'.format(valid))
        r.sleep()

if __name__ == '__main__':
    main()
