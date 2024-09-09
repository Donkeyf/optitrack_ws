#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <geometry_msgs/PoseStamped.h>

class PoseSubscriber {
public:
    PoseSubscriber(ros::NodeHandle& nh) {
        // Initialize the subscriber to the /vrpn_client_node/FASTR/pose topic
        subscriber_ = nh.subscribe("/vrpn_client_node/FASTR/pose", 10, &PoseSubscriber::callback, this);
    }

    void callback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        // Store the received data
        pose_stamped_ = *msg;
    }

    geometry_msgs::PoseStamped getData() {
        return pose_stamped_;
    }

private:
    ros::Subscriber subscriber_;
    geometry_msgs::PoseStamped pose_stamped_;
};

class PosePublisher {
public:
    PosePublisher(ros::NodeHandle& nh) {
        // Initialize the publisher for the /mavros/mocap/pose topic
        mocap_publisher_ = nh.advertise<geometry_msgs::PoseStamped>("/mavros/mocap/pose", 10);
    }

    void sendAttPosMocap(const geometry_msgs::PoseStamped& pose_stamped) {
        // Publish the PoseStamped message
        if (pose_stamped.header.stamp != ros::Time(0)) { // Check if data is available
            mocap_publisher_.publish(pose_stamped);
        }
    }

private:
    ros::Publisher mocap_publisher_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mavros_pose_publisher");
    ros::NodeHandle nh;

    ROS_INFO("Optitrack data is streaming");

    PosePublisher pub(nh);
    PoseSubscriber sub(nh);

    ros::Rate rate(15); // 15 Hz

    while (ros::ok()) {
        ros::spinOnce(); // Process incoming messages

        geometry_msgs::PoseStamped pose_stamped = sub.getData();

        // Print received data
        if (pose_stamped.header.stamp != ros::Time(0)) { // Check if data is available
            ROS_INFO("x = %.3f, y = %.3f, z = %.3f",
                     pose_stamped.pose.position.x,
                     pose_stamped.pose.position.y,
                     pose_stamped.pose.position.z);
        }

        pub.sendAttPosMocap(pose_stamped);

        // TODO for SLAM later
        // mavros_msgs::Mavlink msg;
        // fill in required mavlink msg data

        rate.sleep();
    }

    return 0;
}
