#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/ApproximateTime.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Define the type of synchronization policy you want to use
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Imu> SyncPolicy;

class DataSynchronizer
{
public:
    DataSynchronizer(ros::NodeHandle& nh)
    {
        // Initialize subscribers
        image_sub_.subscribe(nh, "/camera/image_raw", 1);
        imu_sub_.subscribe(nh, "/imu/data", 1);

        // Initialize synchronized subscriber
        sync_.reset(new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), image_sub_, imu_sub_));
        sync_->registerCallback(boost::bind(&DataSynchronizer::callback, this, _1, _2));
    }

private:
    void callback(const sensor_msgs::ImageConstPtr& image_msg, const sensor_msgs::ImuConstPtr& imu_msg)
    {
        try
        {
            // Convert ROS image message to OpenCV image
            cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

            // Process the image and IMU data
            cv::Mat img = cv_image->image;
            ROS_INFO("Received synchronized data: Image Timestamp: %f, IMU Timestamp: %f", 
                     image_msg->header.stamp.toSec(), imu_msg->header.stamp.toSec());

            // Example processing (display the image)
            cv::imshow("Image", img);
            cv::waitKey(1);

            // You can process IMU data here as well
            ROS_INFO("IMU Orientation: [%.2f, %.2f, %.2f, %.2f]", 
                     imu_msg->orientation.x, imu_msg->orientation.y, imu_msg->orientation.z, imu_msg->orientation.w);

        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }
    }

    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub_;
    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "data_synchronizer");
    ros::NodeHandle nh;

    DataSynchronizer ds(nh);

    ros::spin();

    return 0;
}
