import rospy
import time
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL, SetMode

def arm(do_arm):
    
    try:
        arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        resp = arming(do_arm)
        
        rospy.loginfo(resp)

    except rospy.ServiceException as e:
        rospy.logwarn(e)

def set_home_position(lat, lon, alt):
    current_gps = False 
    yaw = 0
    #Set lat, lon, alt as 0,0,0 for reference position. Thus, home is set to wherever 
    # drone is located upon running this node - and this is 0,0,0 in the global frame
    latitude = 0 
    longitude = 0
    altitude = 0
    
    try:
        set_home = rospy.ServiceProxy('/mavros/cmd/set_home', CommandHome)
        response = set_home(current_gps,yaw,latitude,longitude,altitude)

        rospy.loginfo(response)
    
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)

def set_mode(base_mode, custom_mode=''):
    
    try:
        set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        response = set_mode_service(base_mode, custom_mode)
        
        rospy.loginfo("Mode set response: %s", response)

    except rospy.ServiceException as e:
        rospy.logerr("SetMode service call failed: %s", e)

def takeoff(lat,long,alt):
    
    min_pitch = 0
    yaw = 0
    #Set lat, lon and alt as 0,0, x to move x units up. 
    lat = 0
    long = 0
    alt = 1

    try:
        takingoff = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        response = takingoff(min_pitch,yaw,lat,long,alt)

        rospy.loginfo(response)

    except rospy.ServiceException as e:
        rospy.logwarn(e)


if __name__ == '__main__':
    rospy.init_node("arm_demo")

    rospy.wait_for_service("/mavros/cmd/set_home")
    set_home_position(0,0,0)

    time.sleep(5)

    rospy.wait_for_service('/mavros/set_mode')
    set_mode(base_mode=88)

    time.sleep(5)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arm(True)
    
    time.sleep(5)

    rospy.wait_for_service("/mavros/cmd/takeoff")
    takeoff(0,0,1)

    # rospy.spin()
