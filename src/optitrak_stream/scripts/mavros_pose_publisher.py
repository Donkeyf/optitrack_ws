#!usr/bin/env python3
import rospy
from mavros_msgs.msg import Mavlink
from geometry_msgs.msg import PoseStamped

class PoseSubscriber:
    def __init__(self):
        self.X = None  # Variable to store the received data
        self.Y = None  # Variable to store the received data
        self.Z = None  # Variable to store the received data
        # Subscriber to a topic
        self.subscriber = rospy.Subscriber("/vrpn_client_node/Robot_1/pose", PoseStamped, self.callback)
    
    def callback(self, data):
        # Store the data in the variable
        self.X = data.pose.position.x
        self.Y = data.pose.position.y
        self.Z = data.pose.position.z
        self.Q = data.pose.orientation

    def get_data(self):
        return self.received_data

if __name__ == '__main__':
    rospy.init_node("mavros_pose_publisher")
    rospy.loginfo("optitrak data is streaming")

    pub = rospy.Publisher("/mavlink/to", Mavlink, queue_size = 10)
    sub = PoseSubscriber()
    

    #refresh rate at which information is published
    rate = rospy.Rate(15)

    while not rospy.is_shutdown():
        #TODO stream optitrak data to RPi

        print(f'x = {sub.X}, y = {sub.Y}, z = {sub.Z}, q = {sub.Q}')

        msg = Mavlink()
        #TODO fill in required mavlink msg data

        pub.publish(msg)

        rate.sleep()