#include <iostream.h>
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;
using namespace cv;

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  ROS_INFO("Seq: [%d]", msg->header.seq);
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}


int main(int argc,char** argv){
	ros::init(argc,argv,"Odometry");
	ros::NodeHandle nh;

	VideoCapture cap(0); // open the default camera
    if(!cap.isOpened())  // check if we succeeded
        return -1;

    //Mat original;




    namedWindow("original",1);
    for(;;)
    {
        Mat frame;
        cap >> frame; // get a new frame from camera
        imshow("original", frame);
        ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);
        if(waitKey(30) >= 0) break;
    }
    ros::spin();
    
    return 0;
}
