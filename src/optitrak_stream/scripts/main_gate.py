#!usr/bin/env python3
import rospy
import time
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

    def get__pose_NED(self, data):
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
#VECTOR FUNCTIONS
#########################################################################################################################
def rotate_quaternion(p1, p2):
        v = [p1.pose.position.x, p1.pose.position.y, p1.pose.position.z]
        q = [p2.pose.orientation.x, p2.pose.orientation.y, p2.pose.orientation.z, p2.pose.orientation.w]
        
        # rotates a vector with a quaternion, returning unit vector
        u = (q[0]**2 - np.linalg.norm(q[1:])**2)*v + 2*np.dot(q[1:],v)*q[1:] + 2*q[0]*np.cross(q[1:],v)
        return u
        
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



    #ros publishing rate
    rate = rospy.Rate(15)

    while not rospy.is_shutdown():
        if drone_sub.pose_stamped != None:
            rospy.loginfo(f'x = {drone_sub.pose_stamped.pose.position.x:.3f}, y = {drone_sub.pose_stamped.pose.position.y:.3f}, z = {drone_sub.pose_stamped.pose.position.z:.3f}')

        #Send data to ardupilot
        drone_pub.publish(drone_sub.pose_stamped)


        rate.sleep()