/*

*/

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/common/float_image_utils.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <point_coloring.h>

void callback(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& camera, const sensor_msgs::PointCloud2::ConstPtr& lidar)
{
  ROS_INFO("Hello world!!");
}

void oneback(const sensor_msgs::Image::ConstPtr& image)
{
  ROS_INFO("one");
}

void twoback(const sensor_msgs::PointCloud2::ConstPtr& lidar)
{
  ROS_INFO("two");

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "points_matching_image");

  ros::NodeHandle nh;

  // message_filters::Subscriber<sensor_msgs::PointCloud2> image_sub(nh, "/image", 10);
  // message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/lidar", 10);


  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/image", 1);

  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/lidar", 1);

  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_sub(nh, "/camera_info", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> sync;

  // image_sub.registerCallback(oneback);

  // lidar_sub.registerCallback(twoback);

  message_filters::Synchronizer<sync> sub_sync(sync(1), image_sub, camera_sub, lidar_sub);

  sub_sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  // sub.registerCallback(oneback);


  ROS_INFO("good morning");

  ros::spin();
}
