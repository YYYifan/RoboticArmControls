#!/usr/bin/env python
import rospy
from geometry_msgs.msg import (
    Pose,
    Point,
    Quaternion,
)

from control_msgs.msg import (
    FollowJointTrajectoryActionResult,
)

import sys

count = 0
pub = rospy.Publisher("cartesian_goal", Pose, queue_size=10)
poses = [
        Pose(
                position=Point(
                    x=0.86571251362,
    		    y=-0.386556379937,
    		    z=0.193999082242,
                ),
                orientation=Quaternion(
                    x=0.0787102909154,
    		    y=0.995078134807,
    		    z=-0.00457571797974,
    		    w=0.0600271483378,
                ),
            ),
        Pose(
                position=Point(
    		    x=1.02415943981,
         	    y=-0.41299261343,
    		    z=0.814530644453,
                ),
                orientation=Quaternion(
                    x=0.0462063703312,
    		    y=0.579922296894,
    		    z=-0.0645894720726,
    		    w=0.810791774134,

                ),
            ),
#xinjiadelianggezhongjiandian
	Pose(
                position=Point(
    		    x=1.10500784377,
         	    y=-0.417398991307,
    		    z=0.534918374478,
                ),
                orientation=Quaternion(
                    x=0.0482942854592,
    		    y=0.692874839531,
    		    z=-0.0571713801936,
    		    w=0.717163546218,

                ),
            ),
	Pose(
                position=Point(
    		    x=1.1071318456,
         	    y=-0.417662283353,
    		    z=0.281451282299,
                ),
                orientation=Quaternion(
                    x=0.0531843810035,
    		    y=0.778762205342,
    		    z=-0.0507445819271,
    		    w=0.622997461113,

                ),
            ),
#-----------------------
        Pose(
                position=Point(
                    x=0.996730222608,
    		    y=-0.428707004222,
    		    z=-0.0607415637869,
                ),
                orientation=Quaternion(
                    x=0.0302801016895,
    		    y=0.879050929744,
    		    z=-0.0747726058101,
    		    w=0.469852780962,
                ),
            ),
#--------------xinjianjieduansan------
	Pose(
                position=Point(
                    x=0.971946186097,
    		    y=-0.143503600188,
    		    z=-0.09612874566,
                ),
                orientation=Quaternion(
                    x=-0.0658491189762,
    		    y=0.893383614817,
    		    z=0.0239233755102,
    		    w=0.44379869582,
                ),
            ),
	Pose(
                position=Point(
                    x=0.777777668517,
    		    y=0.313884497079,
    		    z=-0.019622758787,
                ),
                orientation=Quaternion(
                    x=-0.305909116713,
    		    y=0.841976270125,
    		    z=0.143390499799,
    		    w=0.420636110463,
                ),
            ),
#--------------------------------------
        Pose(
                position=Point(
                    x=0.867087413752,
    		    y=0.178915330492,
    		    z=-0.120724159166,
                ),
                orientation=Quaternion(
                    x=-0.230511081023,
    		    y=0.866396777408,
    		    z=0.105326538887,
    		    w=0.430264553303,
                ),
            ),
        ]

def callback(data):
    global count
    rospy.loginfo(poses[count])
    pub.publish(poses[count])
    count += 1;
    if (count >= 8):
        print "achieve the goal sucessfully!exiting..."
        rospy.signal_shutdown(0)
        

def main():
    rospy.init_node('kitty_cartesian_publisher', anonymous=True)
    callback(0)
    rospy.Subscriber("/robot/limb/right/follow_joint_trajectory/result", FollowJointTrajectoryActionResult, callback)
    rospy.spin();
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
