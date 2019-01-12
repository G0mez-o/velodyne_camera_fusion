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

class num_pub{
    private:
        ros::NodeHandle nh;
        message_filters::Subscriber<std_msgs::Int8> obj_num_sub;

    public:
  //コンストラクタ
        num_pub();
        void Callback(const sensor_msgs::Image::ConstPtr&, const sensor_msgs::CameraInfo::ConstPtr&, const sensor_msgs::PointCloud2::ConstPtr&);
        void pub_callback(const std_msgs::Int8::ConstPtr&);
};

num_pub::num_pub()
    : nh("~"),
      obj_num_sub(nh, "/darknet_ros/found_object", 1)
{
    obj_num_sub(num_pub::pub_callback);
}

void num_pub::pub_callback(std_msgs::Int8::ConstPtr& msg)
{
  ros::Publisher nu_pub;
  ros::NodeHandle nhh("~~");
  nu_pub = nhh.advertise<sensor_fusion::object_num>("/obje_num", 10);
  sensor_fusion::object_num num;
  num.header.stamp = ros::Time::now();
  num.num = msg->data;
  nu_pub.publish(num);
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

  num_pub pu;

  ros::spin();
}
