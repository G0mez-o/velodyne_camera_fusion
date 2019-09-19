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

#include <point_coloring.h>


ros::Publisher pub;


void callback(const sensor_msgs::Image image)
{
  sensor_msgs::CameraInfo cinfo;
  cinfo.header.stamp = image.header.stamp;
  cinfo.header.frame_id = "occam";
  cinfo.height = 480;
  cinfo.width = 752;
  cinfo.distortion_model = "plumb_bob";
  cinfo.D = {-0.3160332143306732, 0.11990676820278168, -0.000810914090834558, -0.0005695929867215455, -0.02259594388306141};
  cinfo.K = {478.47052001953125, 0.0, 365.0530700683594, 0.0, 478.2624206542969, 191.83615112304688, 0.0, 0.0, 1.0};
  cinfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  cinfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  cinfo.P = {470.0, -0.005907885264605284, 125.0, 0.0, 0.005914974492043257, 580.9821782112122, -85.0, 0, 0.008313752710819244, -0.0008770190179347992, 0.9999650716781616, -47.651424407958984};
//  cinfo.P = {465.0, -0.005907885264605284, 25.0, 0.0, 0.005914974492043257, 580.9821782112122, -85.0, 0, 0.008313752710819244, -0.0008770190179347992, 0.9999650716781616, -47.651424407958984};
//  cinfo.P = {500.0, -0.005907885264605284, 25.0, 0.0, 0.005914974492043257, 580.9821782112122, -85.0, 0, 0.008313752710819244, -0.0008770190179347992, 0.9999650716781616, -47.651424407958984};
//  cinfo.P = {470.0, -0.005907885264605284, 0.0, 0.0, 0.005914974492043257, 580.9821782112122, -85.0, 0, 0.008313752710819244, -0.0008770190179347992, 0.9999650716781616, -47.651424407958984};
//  cinfo.P = {470.0, -0.005907885264605284, 15.0, 0.0, 0.005914974492043257, 580.9821782112122, -85.0, 0, 0.008313752710819244, -0.0008770190179347992, 0.9999650716781616, -47.651424407958984};
 // cinfo.P = {999.9479651451111, -0.005907885264605284, -8.318792097270489, 0, 0.005914974492043257, 999.9821782112122, 0.8278567111119628, 0, 0.008313752710819244, -0.0008770190179347992, 0.9999650716781616, -47.651424407958984};
  cinfo.binning_x = 1;
  cinfo.binning_y = 1;
  cinfo.roi.x_offset = 0;
  cinfo.roi.y_offset = 0;
  cinfo.roi.height = 0;
  cinfo.roi.width = 0;
  cinfo.roi.do_rectify = false;

  pub.publish(cinfo);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "camera_info_publisher");

  ros::NodeHandle nh;

  pub = nh.advertise<sensor_msgs::CameraInfo>("/camera_info", 1);

  ROS_INFO("declare node name!!");

  ros::Subscriber obj_num_sub = nh.subscribe("/occam/image0", 1, callback);
  ros::spin();

}
