import rospy
from mavros_msgs.srv import SetMode


def set_mode(mode):
    try:
        set_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)
        resp = set_mode(mode)
        rospy.loginfo(resp)
    except rospy.ServiceException as e:
        rospy.logwarn(e)
    


if __name__ == '__main__':
    rospy.init_node("mode_change_demo")
    rospy.wait_for_service("mavros/set_mode")
    set_mode(88)