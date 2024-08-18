#!usr/bin/env python3
import rospy
from mavros_msgs.msg import Mavlink
from geometry_msgs.msg import PoseStamped

class PoseSubscriber:
    def __init__(self):
        self.pose_stamped = None
        # Subscriber to a topic
        self.subscriber = rospy.Subscriber("/vrpn_client_node/FASTR/pose", PoseStamped, self.callback)
    
    def callback(self, data):
        # Store the data in the variable
        self.pose_stamped = data
        temp = data.pose.position.x
        # Swapping because Ardupilot X is positive North, but Optitrack y is positive North
        self.pose_stamped.pose.position.x = -1.0 * data.pose.position.y # x is E
        self.pose_stamped.pose.position.y = temp # y is N

    def get_data(self):
        return self.received_data
    
class PosePublisher:
    def __init__(self):
        self.mocap_publisher = rospy.Publisher('/mavros/mocap/pose', PoseStamped, queue_size=10)
        #refresh rate at which information is published
        

    def send_att_pos_mocap(self,pose_stamped):
        # publish the PoseStamped message
        if (pose_stamped != None):
            self.mocap_publisher.publish(pose_stamped)

if __name__ == '__main__':
    rospy.init_node("mavros_pose_publisher")
    rospy.loginfo("optitrak data is streaming")

    pub = PosePublisher()
    sub = PoseSubscriber()

    rate = rospy.Rate(15)

    while not rospy.is_shutdown():
        #TODO stream optitrak data to RPi

        if sub.pose_stamped != None:
            print(f'x = {sub.pose_stamped.pose.position.x:.3f}, y = {sub.pose_stamped.pose.position.y:.3f}, z = {sub.pose_stamped.pose.position.z:.3f}')

        pub.send_att_pos_mocap(sub.pose_stamped)

        # TODO for SLAM later
        # msg = Mavlink()
        # fill in required mavlink msg data

        rate.sleep()