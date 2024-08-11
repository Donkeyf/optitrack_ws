#!usr/bin/env python3
import rospy
from mavros_msgs.msg import Mavlink
from geometry_msgs.msg import PoseStamped

def receive_pose(pose):
    print(pose)

class OptitrackSubscriber:
    def __init__(self):
        self.received_data = None  # Variable to store the received data
        # Subscriber to a topic
        self.subscriber = rospy.Subscriber("/vrpn_client_node/Robot_1/pose", PoseStamped, receive_pose)
    
    def callback(self, data):
        # Store the data in the variable
        self.received_data = data.data
        rospy.loginfo(f"Data received: {self.received_data}")

    def get_data(self):
        return self.received_data

if __name__ == '__main__':
    rospy.init_node("mavros_pose_publisher")
    rospy.loginfo("optitrak data is streaming")

    pub = rospy.Publisher("/mavlink/to", Mavlink, queue_size = 10)
    opt_sub = OptitrackSubscriber()
    
    X = opt_sub.get_data().pose.position.x
    Y = opt_sub.get_data().pose.position.y
    Z = opt_sub.get_data().pose.position.z

    #refresh rate at which information is published
    rate = rospy.Rate(15)

    while not rospy.is_shutdown():
        #TODO stream optitrak data to RPi

        print(f'x = {X}, y = {Y}, z = {Z}')

        msg = Mavlink()
        #TODO fill in required mavlink msg data

        pub.publish(msg)

        rate.sleep()