import rospy
from mavros_msgs.srv import CommandBool

def arm(do_arm):
    try:
        arming = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        resp = arming(do_arm)
        rospy.loginfo(resp)

    except rospy.ServiceException as e:
        rospy.logwarn(e)

if __name__ == '__main__':
    rospy.init_node("arm_demo")
    rospy.wait_for_service("mavros/cmd/arming")
    arm(True)
    # rospy.spin()