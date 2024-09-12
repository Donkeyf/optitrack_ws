#!usr/bin/env python3
import rospy
from mavros_msgs.msg import Mavlink
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool

def arm(do_arm):
    resp = False
    try:
        arming = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        resp = arming(do_arm)
        rospy.loginfo(resp)
        

    except rospy.ServiceException as e:
        rospy.logwarn(e)
    return resp

class PoseSubscriber:
    def __init__(self):
        self.pose_stamped = None
        # Subscriber to a topic
        self.subscriber = rospy.Subscriber("/vrpn_client_node/FASTR/pose", PoseStamped, self.callback)
    
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

    armed = False

    while not rospy.is_shutdown():
        #TODO stream optitrak data to RPi

        if sub.pose_stamped != None:
            print(f'x = {sub.pose_stamped.pose.position.x:.3f}, y = {sub.pose_stamped.pose.position.y:.3f}, z = {sub.pose_stamped.pose.position.z:.3f}')

        pub.send_att_pos_mocap(sub.pose_stamped)
<<<<<<< HEAD
=======

        # Arm if not armed
        if not armed:
            rospy.wait_for_service("/mavros/cmd/arming")
            resp = arm(True)
>>>>>>> de3290bd28b5d9435d9f25b66c35d45494006246
        
        # Arm if not armed
        rospy.wait_for_service("/mavros/cmd/arming")
        arm(True)
        #print(f'armed = {armed}')
        #if not armed:
        #    rospy.wait_for_service("/mavros/cmd/arming")
        #    resp = arm(True)
        #    print(resp)
        
        #if resp:
        #    armed = True

        # TODO for SLAM later
        # msg = Mavlink()
        # fill in required mavlink msg data

        rate.sleep()
