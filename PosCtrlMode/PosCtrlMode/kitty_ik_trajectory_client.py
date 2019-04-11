#!/usr/bin/env python
import rospy
import struct
import sys
from copy import copy
#--------------ik_service_client---------#
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

#------------joint_trajectory_client-----#

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import baxter_interface

from baxter_interface import CHECK_VERSION

class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]

#---------------modify from ik_service_client-----------#
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + " I subscribe %s", data)
    ns = "ExternalTools/" + "right" + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    ps = PoseStamped(header=hdr, pose=data)
    ikreq.pose_stamp.append(ps)
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                   }.get(resp_seeds[0], 'None')
        print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
              (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))

#-------------------modify from joint_trajectory_client-----#
        traj = Trajectory("right")
        rospy.on_shutdown(traj.stop)
        # Command Current Joint Positions first
        limb_interface = baxter_interface.limb.Limb("right")
        current_angles = [limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0)

        print(resp.joints[0].position)
        p1 = resp.joints[0].position
        traj.add_point(p1, 7.0)
        traj.start()
        traj.wait(15.0)
        print("Exiting - Joint Trajectory Action Complete")
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")

    return 0

def main():
    print("Initializing node... ")
    rospy.init_node('kitty_ik_trajectory_client', anonymous=True)
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    print("Enabling robot... ")
    rs.enable()
    print("Moving to neutral pose...")
    right_arm = baxter_interface.limb.Limb("right")
    right_arm.move_to_neutral()
    print("OK!")
    print("subscribing...")
    rospy.Subscriber("cartesian_goal", Pose, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == "__main__":
    main()
