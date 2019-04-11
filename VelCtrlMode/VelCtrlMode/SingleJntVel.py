#!/usr/bin/env python

import argparse
import math
import random

import rospy

from std_msgs.msg import (
    UInt16,
)

import baxter_interface

from baxter_interface import CHECK_VERSION


class Wobbler(object):

    def __init__(self):
        """
        'Wobbles' both arms by commanding joint velocities sinusoidally.
        """
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16, queue_size=10)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_joint_names = self._left_arm.joint_names()
        self._right_joint_names = self._right_arm.joint_names()

        # control parameters
        self._rate = 500.0  # Hz

        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # set joint state publishing to 500Hz
        self._pub_rate.publish(self._rate)

    def _reset_control_modes(self):
        rate = rospy.Rate(self._rate)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self._right_arm.exit_control_mode()
            self._pub_rate.publish(100)  # 100Hz default joint state rate
            rate.sleep()
        return True

    def set_neutral(self):
        """
        Sets both arms back into a neutral pose.
        """
        print("Moving to neutral pose...")
        self._left_arm.move_to_neutral()
        self._right_arm.move_to_neutral()

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        self.set_neutral()
        if not self._init_state:
            print("Disabling robot...")
            self._rs.disable()
        return True

    def wobble(self, limb):
        self.set_neutral()
        """
        Performs the wobbling of both arms.
        """
        rate = rospy.Rate(self._rate)
        start = rospy.Time.now()

        def make_v_func():
            """
            returns a randomly parameterized cosine function to control a
            specific joint.
            """
#set fixed frequency and amplitude-------------------------
            period_factor = 0.5
            amplitude_factor = 1
#----------------------------------------------------------
            def v_func(elapsed):
                w = period_factor * elapsed.to_sec()
                return amplitude_factor * math.cos(w * 2 * math.pi)
            return v_func
#----------------single joint------------
        v_func_limb = make_v_func()

        def make_cmd(elapsed):
            return dict({limb: v_func_limb(elapsed), })
#----------------------------------------------------------
        print("Wobbling. Press Ctrl-C to stop...")
        while not rospy.is_shutdown():
            try:
            	self._pub_rate.publish(self._rate)
            	elapsed = rospy.Time.now() - start
            	cmd = make_cmd(elapsed)
            	self._right_arm.set_joint_velocities(cmd)
            	rate.sleep()
            except (rospy.ROSException), e:
            	return 0;


def main():
    """RSDK Joint Velocity Example: Wobbler

    Commands joint velocities of randomly parameterized cosine waves
    to each joint. Demonstrates Joint Velocity Control Mode.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-j', '--joint', required=True, choices=['right_s0', 'right_s1', 'right_e0', 'right_e1', 'right_w0', 'right_w1', 'right_w2'],
        help='send joint trajectory to which joint'
    )
    args = parser.parse_args(rospy.myargv()[1:])
    
    limb = args.joint

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_velocity_wobbler")

    wobbler = Wobbler()
    rospy.on_shutdown(wobbler.clean_shutdown)
    wobbler.wobble(limb)

    print("Done.")

if __name__ == '__main__':
    main()
