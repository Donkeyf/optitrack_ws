#!usr/bin/env python3
import rospy
import asyncio
import numpy as np
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, CommandHome, CommandTOL, SetMode, CommandLong



#########################################################################################################################
#Publisher Subscriber Functions
#########################################################################################################################
class PoseSubscriber:
    def __init__(self, topic):
        self.pose_stamped = None
        # Subscriber to a topic
        self.subscriber = rospy.Subscriber(topic, PoseStamped, self.get__pose_NED)

    def callback(self, data):
        if self.pose_stamped == None:
            self.pose_stamped = data
        # Store the data in the variable
        temp = data.pose.position.x
        # Swapping because Ardupilot X is positive North, but Optitrack y is positive North
        self.pose_stamped.pose.position.x = data.pose.position.y 
        self.pose_stamped.pose.position.y = -1.0 * temp
        self.pose_stamped.pose.position.z = data.pose.position.z
        self.pose_stamped.pose.orientation = data.pose.orientation    

class PosePublisher:
    def __init__(self, topic):
        self.mocap_publisher = rospy.Publisher(topic, PoseStamped, queue_size=10)

    def publish(self, pose_stamped):
        # publish the PoseStamped message
        if (pose_stamped != None):
            self.mocap_publisher.publish(pose_stamped)



#########################################################################################################################
#Arming Functions
#########################################################################################################################
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


def takeoff_local(x, y, z):
    min_pitch = 0
    yaw = 0
    ascend = 0.2
    cmd = 24

    try:
        takingoff = rospy.ServiceProxy("/mavros/cmd/command", CommandLong)
        #comand, confirmation, param1(pitch), param(empty), param3(ascend), param4(yaw), param5(y), param6(x), param7(z) 
        response = takingoff(cmd, 10, min_pitch, None, ascend, yaw, y, x, z)

        rospy.loginfo(response)

    except rospy.ServiceException as e:
        rospy.logwarn(e)


#########################################################################################################################
#VECTOR FUNCTIONS
#########################################################################################################################
def rotate_quaternion(p1, p2):
        #calculate quaternion rotation to calculate intermediate waypoint (Check with Xin)
        v = [p1.pose.position.x, p1.pose.position.y, p1.pose.position.z]
        q = [p2.pose.orientation.x, p2.pose.orientation.y, p2.pose.orientation.z, p2.pose.orientation.w]
        
        # rotates a vector with a quaternion, returning unit vector
        u = (q[0]**2 - np.linalg.norm(q[1:])**2)*v + 2*np.dot(q[1:],v)*q[1:] + 2*q[0]*np.cross(q[1:],v)
        return u
        

#########################################################################################################################
#ASYNC AND SUPPORT FUNCTIONS
#########################################################################################################################
async def drone_setup(x, y, z, gate_sub, drone_pose, drone_sub):
    #run drone setup and waypoint setting asynchronously with main sub/pub loop
    await asyncio.sleep(2)

    #set home position
    rospy.wait_for_service("/mavros/cmd/set_home")
    set_home_position(0,0,0)

    #set mode to guided
    rospy.wait_for_service('/mavros/set_mode')
    set_mode(base_mode=88)
    await asyncio.sleep(2)
    
    #arm copter
    rospy.wait_for_service("/mavros/cmd/arming")
    arm(True)
    await asyncio.sleep(5)

    #TODO make sure works
    #give takeoff command
    rospy.wait_for_service("/mavros/cmd/command")
    takeoff_local(x, y, z)
    await asyncio.sleep(2)

    int_waypoint = []

    #get fixed current pose to do calculations, so that angle is accurate
    curr_pose = drone_sub.pose_stamped
    p_x = curr_pose.pose.position.x
    p_y = curr_pose.pose.position.y
    p_z = curr_pose.pose.position.z

    #calculate intermediate waypoint
    u = rotate_quaternion(curr_pose, gate_sub.pose_stamped)
    p_x += 1 * u[0]        #change factor of u to change distance from gate inter waypoint is, eg 1 = 1m
    p_y += 1 * u[1]
    p_z += 1 * u[2]

    #publish waypoint
    gate_pub.publish(curr_pose.pose_stamped)

    #check if drone has reached waypoint
    while True:
        wp_x = drone_pose.pose_stamped.pose.position.x
        wp_y = drone_pose.pose_stamped.pose.position.y
        wp_z = drone_pose.pose_stamped.pose.position.z

        #check if drone is within 20cm of waypoint
        if (p_x + 0.2 < wp_x < p_x - 0.2 and p_y + 0.2 < wp_y < p_y - 0.2 and p_z + 0.2 < wp_z < p_z - 0.2):
            break
        await asyncio.sleep(0.2)

    # TODO Make sure the drone goes THRU the gate and not just stop at the gate
    #publish final waypoint
    gate_pub.publish(gate_sub.pose_stamped)

    g_x = gate_sub.pose_stamped.pose.position.x
    g_y = gate_sub.pose_stamped.pose.position.y
    g_z = gate_sub.pose_stamped.pose.position.z

    while True:
        wp_x = drone_pose.pose_stamped.pose.position.x
        wp_y = drone_pose.pose_stamped.pose.position.y
        wp_z = drone_pose.pose_stamped.pose.position.z

        if (g_x + 0.2 < wp_x < g_x - 0.2 and g_y + 0.2 < wp_y < g_y - 0.2 and g_z + 0.2 < wp_z < g_z - 0.2):
            break
        await asyncio.sleep(0.2)

    # TODO add land function

    return


async def EKF_loop(drone_pub, drone_sub):
    #ros publishing rate
    rate = rospy.Rate(15)

    while not rospy.is_shutdown():
        if drone_sub.pose_stamped != None:
            rospy.loginfo(f'x = {drone_sub.pose_stamped.pose.position.x:.3f}, y = {drone_sub.pose_stamped.pose.position.y:.3f}, z = {drone_sub.pose_stamped.pose.position.z:.3f}')

        #Send data to ardupilot
        drone_pub.publish(drone_sub.pose_stamped)

        rate.sleep()
    
    return

async def main(x, y, z, gate_sub, drone_pose, drone_sub, drone_pub):
    #asynchronously run both functions
    await asyncio.gather(drone_setup(x, y, z, gate_sub, drone_pose, drone_sub), EKF_loop(drone_pub, drone_sub))
    return

#########################################################################################################################
#MAIN FUNCTION
#########################################################################################################################
if __name__ == '__main__':
    rospy.init_node("main_gate")
    rospy.loginfo("optitrak data is streaming")

    #set pub and subs
    drone_pub = PosePublisher("/mavros/mocap/pose")
    gate_sub = PoseSubscriber("/vrpn_client_node/GATE/pose")
    drone_sub = PoseSubscriber("/vrpn_client_node/FASTR/pose")
    gate_pub = PosePublisher("/mavros/setpoint_position/local")
    drone_pose = PoseSubscriber("/mavros/local_position/pose")

    #TODO read current position and set x, y to current position and z to desired takeoff altitude
    x = 0
    y = 0
    z = 0

    asyncio.run(main(x, y, z, gate_sub, drone_pose, drone_sub, drone_pub))






