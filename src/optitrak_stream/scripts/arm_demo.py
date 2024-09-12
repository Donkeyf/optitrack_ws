import rospy
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
import time
from mavros_msgs.srv import CommandLong
from mavros_msgs.msg import CommandCode
from mavros_msgs.srv import CommandHome

def arm(do_arm):
    try:
        arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        resp = arming(do_arm)
        rospy.loginfo(resp)

    except rospy.ServiceException as e:
        rospy.logwarn(e)

def set_home_position(lat, lon, alt):
    rospy.wait_for_service('/mavros/cmd/set_home')
    try:
        set_home = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
        response = set_home(current_gps=False,yaw=0,latitude=lat,longitude=lon,altitude=alt)
        rospy.loginfo(response)
        if response.success:
            rospy.loginfo("Home position set successfully.")
        else:
            rospy.logwarn("Failed to set home position.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def takeoff(x,y,z):
    min_pitch = 0
    offset = 0
    rate = 0.1
    yaw = 0
    position = [x,y,z]

    lat = 0
    long = 0
    alt = 1

    try:
        takingoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        resp = takingoff(min_pitch,yaw,lat,long,alt)
        #resp = takingoff(min_pitch,offset,rate,yaw,position)
        rospy.loginfo(resp)

    except rospy.ServiceException as e:
        rospy.logwarn(e)

def send_takeoff_cmd():
    rospy.wait_for_service('/mavros/cmd/command')

    try:
        command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)
        resp = command_service(
            command=CommandCode.NAV_TAKEOFF_LOCAL,
            param1 = 0,
            param2 = 0,
            param3 = 0,
            param4 = 0,
            param5 = 0, # x
            param6 = 0, # y
            param7 = 1 # z
        )
        rospy.loginfo(resp)
        if resp.success:
            rospy.loginfo("Takeoff command sent successfully")
        else:
            rospy.logwarn("Failed to send takeoff command.")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':
    rospy.init_node("arm_demo")
    
    set_home_position(0,0,0)

    time.sleep(5)

    rospy.wait_for_service("/mavros/cmd/arming")
    arm(True)
    time.sleep(5)

    #rospy.wait_for_service("/mavros/cmd/takeoff")
    #takeoff(0,0,1)
    send_takeoff_cmd()
    # rospy.spin()
