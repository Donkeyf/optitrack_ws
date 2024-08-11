#!usr/bin/env python3
import rospy
from mavros_msgs.msg import Mavlink
from geometry_msgs.msg import PoseStamped

class PoseSubscriber:
    def __init__(self):
        self.pose_stamped = None
        self.X = None  # Variable to store the received data
        self.Y = None  # Variable to store the received data
        self.Z = None  # Variable to store the received data
        self.Q = []
        # Subscriber to a topic
        self.subscriber = rospy.Subscriber("/vrpn_client_node/FASTR/pose", PoseStamped, self.callback)
    
    def callback(self, data):
        # Store the data in the variable
        self.pose_stamped = data
        self.X = data.pose.position.x
        self.Y = data.pose.position.y
        self.Z = data.pose.position.z
        self.Q = [data.pose.orientation.x]
        self.Q.append(data.pose.orientation.y)
        self.Q.append(data.pose.orientation.z)
        self.Q.append(data.pose.orientation.w)

    def get_data(self):
        return self.received_data
    
class PosePublisher:
    def __init__(self):
        self.mocap_publisher = rospy.Publisher('/mavros/mocap/pose', PoseStamped, queue_size=10)
        #refresh rate at which information is published
        

    def send_att_pos_mocap(self,pose_stamped):
        # publish the PoseStamped message
        self.mocap_publisher.publish(pose_stamped)

if __name__ == '__main__':
    rospy.init_node("mavros_pose_publisher")
    rospy.loginfo("optitrak data is streaming")

    pub = PosePublisher()
    sub = PoseSubscriber()

    rate = rospy.Rate(15)

    while not rospy.is_shutdown():
        #TODO stream optitrak data to RPi

        print(f'x = {sub.X}, y = {sub.Y}, z = {sub.Z}, q = {sub.Q}')

        pub.send_att_pos_mocap(sub.pose_stamped)

        # TODO for SLAM later
        # msg = Mavlink()
        # fill in required mavlink msg data

        rate.sleep()