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

ros::Publisher pub;

void callback(const sensor_msgs::CameraInfo::ConstPtr& camera, const sensor_msgs::Image::ConstPtr& image)
{
  std::cout<<"cv_bridge"<<std::endl;
  cv_bridge::CvImageConstPtr cv_img_ptr;
  try{
    cv_img_ptr = cv_bridge::toCvShare(image);
  }catch (cv_bridge::Exception& e){
    ROS_INFO("cv_bridge exception: %s", e.what());
    return;
  }
  cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
  cv_image = cv_bridge::toCvShare(image)->image;

  cv::Mat rgb_image;
  cv::cvtColor(cv_image, rgb_image, CV_BGR2RGB);

  cv::Point2d ub(752, 480);

  cv::circle(rgb_image, ub, 10, cv::Scalar(int(255),int(255),int(255)), -1);
  ROS_INFO("one circle show in image!!");
  sensor_msgs::ImagePtr circle_image;
  circle_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
  circle_image->header.frame_id = image->header.frame_id;
  circle_image->header.stamp = ros::Time::now();
  pub.publish(circle_image);
  // image_geometry::PinholeCameraModel cam_model;
  // cam_model.fromCameraInfo(camera);
  // cv::Point3d pt_cv(0.15, 0.15, 0.45);
  // cv::Point2d uv;
  // uv = cam_model.project3dToPixel(pt_cv);
  // ROS_INFO("uv.x: %f && uv.y: %f", uv.x, uv.y);
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

  pub = nh.advertise<sensor_msgs::Image>("/one_point_image", 10);

  // message_filters::Subscriber<sensor_msgs::PointCloud2> image_sub(nh, "/image", 10);
  // message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/lidar", 10);


  message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/image", 1);

  // message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, "/lidar", 1);

  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_sub(nh, "/camera_info", 1);

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CameraInfo, sensor_msgs::Image> sync;

  // image_sub.registerCallback(oneback);

  // lidar_sub.registerCallback(twoback);

  // camera_sub.registerCallback(callback);

  message_filters::Synchronizer<sync> sub_sync(sync(1), camera_sub, image_sub);

  sub_sync.registerCallback(boost::bind(&callback, _1, _2));

  // sub.registerCallback(oneback);


  ROS_INFO("good morning");

  ros::spin();
}
