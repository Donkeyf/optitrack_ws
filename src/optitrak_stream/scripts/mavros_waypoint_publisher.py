import rospy
from mavros_msgs.msg import Mavlink
from geometry_msgs.msg import PoseStamped
import numpy as np

class WaypointSubscriber:
    def __init__(self, topic):
        self.pose_stamped = None
        # Subscriber to a topic
        self.subscriber = rospy.Subscriber(topic, PoseStamped, self.callback)
    
    def callback(self, data):
        # Store the data in the variable
        self.pose_stamped = data
        # Variable assignment
        self.position = np.array([data.pose.position.x, data.pose.position.y, data.pose.position.z])
        self.quaternion = np.array([data.pose.orientation.w, data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z])
                
    def rotate_quaternion(v, q):
        # rotates a vector with a quaternion, returning unit vector
        u = (q[0]**2 - np.linalg.norm(q[1:])**2)*v + 2*np.dot(q[1:],v)*q[1:] + 2*q[0]*np.cross(q[1:],v)
        return u
    
    def get_data(self):
        return self.received_data


class WaypointPublisher:
    def __init__(self, topic):
        self.waypoint_publisher = rospy.Publisher(topic, PoseStamped, queue_size=10)
    
    def send_waypoint(self,waypoint):
        # publish the PoseStamped message
        if (waypoint != None):
            self.waypoint_publisher.publish(waypoint)





if __name__ == "__main__":
    rospy.init_node("mavros_waypoint_publisher")

    sub = WaypointSubscriber('/vrpn_client_node/GATE/pose')         # create subscriber
    pub = WaypointPublisher('mavros/setpoint_position/global')      # create publisher
    
    while not rospy.is_shutdown():
        # use gate pose (subscribed) to find intermediate waypoint, and actual waypoint
        int_waypoint_global = sub.position + 1.5 * sub.rotate_quaternion(np.array([0,1,0]), sub.quaternion)
        waypoint_global = sub.position

        # publish the waypoints
        pub.send_waypoint(int_waypoint_global)
        pub.send_waypoint(waypoint_global)

