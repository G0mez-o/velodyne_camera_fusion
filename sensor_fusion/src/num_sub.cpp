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
#include <sensor_fusion/object_num.h>
#include <std_msgs/Int8.h>

ros::NodeHandle nh;
void pub_callback(std_msgs::Int8 msg)
{
  ros::Publisher num_pub = nh.advertise<sensor_fusion::object_num>("/obje_num", 10);
  sensor_fusion::object_num num;
  num.header.stamp = ros::Time::now();
  num.num = msg.data;
  num_pub.publish(num);
  // image_geometry::PinholeCameraModel cam_model;
  // cam_model.fromCameraInfo(camera);
  // cv::Point3d pt_cv(0.15, 0.15, 0.45);
  // cv::Point2d uv;
  // uv = cam_model.project3dToPixel(pt_cv);
  // ROS_INFO("uv.x: %f && uv.y: %f", uv.x, uv.y);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "object_number_pub");

  ros::Subscriber obj_num_sub = nh.subscribe("/darknet_ros/found_object", 1, pub_callback);

  ros::spin();
}
