
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

#include <jsk_recognition_msgs/BoundingBoxArray.h>

#include <point_coloring.h>

#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>

#include <sensor_fusion/object_datas.h>
#include <sensor_fusion/object_data.h>
#include <sensor_fusion/object_num.h>

class bounding_pub{
    private:
        ros::NodeHandle nh;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes, sensor_fusion::object_num> fusion_bounding_subs;
        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> proj_point_sub;
        message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub;
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_sub;
        message_filters::Subscriber<sensor_fusion::object_num> obj_num_sub;
        message_filters::Synchronizer<fusion_bounding_subs> fusion_bounding_sync;
        ros::Publisher pub;
        ros::Publisher pub_;
        ros::Publisher pub__;

    public:
  //コンストラクタ
        bounding_pub();
        void Callback(const sensor_msgs::Image::ConstPtr&, const sensor_msgs::CameraInfo::ConstPtr&, const sensor_msgs::PointCloud2::ConstPtr&, const darknet_ros_msgs::BoundingBoxes::ConstPtr&, const sensor_fusion::object_num::ConstPtr&);
};

bounding_pub::bounding_pub()
    : nh("~"),
      image_sub(nh, "/occam/image0", 10), proj_point_sub(nh, "/fusion_points", 10), cam_info_sub(nh, "/camera_info", 10), bounding_sub(nh, "/darknet_ros/bounding_boxes", 10), obj_num_sub(nh, "/obje_num", 10),
      fusion_bounding_sync(fusion_bounding_subs(10), image_sub, cam_info_sub, proj_point_sub, bounding_sub, obj_num_sub)

{
    pub = nh.advertise<sensor_msgs::PointCloud2>("/object_points", 10);
    pub_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/object_boxes", 10);
    pub__ = nh.advertise<sensor_fusion::object_datas>("/object_range", 10);
    fusion_bounding_sync.registerCallback(boost::bind(&bounding_pub::Callback, this, _1, _2, _3, _4, _5));
}

void bounding_pub::Callback(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cinfo, const sensor_msgs::PointCloud2::ConstPtr& points, const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes, const sensor_fusion::object_num::ConstPtr& num)
{
  int8_t obj_num = num->num;
  int j = 0;


  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  jsk_recognition_msgs::BoundingBoxArray bboxes;
  jsk_recognition_msgs::BoundingBox bbox;
  sensor_fusion::object_datas datas;
  sensor_fusion::object_data data;
  pcl::fromROSMsg(*points, *cloud);
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(cinfo);
  pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
  msg->header.frame_id = "occam/image0";
  msg->points.resize(20000);
  double param[obj_num][5] = {0};
  std::string objname[obj_num];
  for (int i = 0; i < obj_num; i++)
    {
      objname[i] = boxes->bounding_boxes[i].Class;
    }
  for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pt=cloud->points.begin(); pt<cloud->points.end(); pt++)
    {
      cv::Point3d p_cv((*pt).x, (*pt).y, (*pt).z);
      cv::Point2d pv;
      pv = cam_model.project3dToPixel(p_cv);
      cv::Point2d pv_(pv.x + 240, pv.y + 360);
      for (int i = 0; i < obj_num; i++)
	{
	  if (pv_.x > boxes->bounding_boxes[i].xmin && pv_.x < boxes->bounding_boxes[i].xmax && pv_.y > boxes->bounding_boxes[i].ymin && pv_.y < boxes->bounding_boxes[i].ymax)
	    {
	      param[i][0] += (*pt).x;
	      param[i][1] += (*pt).y;
	      param[i][2] += (*pt).z;
	      param[i][3] += sqrt( pow((*pt).x, 2.0) + pow((*pt).z, 2.0));
	      param[i][4]++;
	      msg->points[j].x = (*pt).x;
	      msg->points[j].y = (*pt).y;
	      msg->points[j].z = (*pt).z;
	      msg->points.push_back(msg->points[j]);
	      j++;
	    }
	}
    }
  for (int i = 0; i < obj_num; i++)
    {
      param[i][0] /= param[i][4];
      param[i][1] /= param[i][4];
      param[i][2] /= param[i][4];
      param[i][3] /= param[i][4];
      bbox.header.frame_id = "occam/image0";
      bbox.pose.position.x =  param[i][0];
      bbox.pose.position.y =  param[i][1];
      bbox.pose.position.z =  param[i][2];
      bbox.pose.orientation.x = 0;
      bbox.pose.orientation.y = 0;
      bbox.pose.orientation.z = 0;
      bbox.dimensions.x = param[i][0];
      bbox.dimensions.y = param[i][1];
      bbox.dimensions.z = param[i][2];
      bboxes.boxes.push_back(bbox);
      data.header.frame_id = "occam/image0";
      data.object_name = objname[i];
      data.x = param[i][0];
      data.y = param[i][1];
      data.z = param[i][2];
      data.range = param[i][3];
      datas.datas.push_back(data);
    }
  pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
  pub.publish(msg);
  pub_.publish(bboxes);
  pub__.publish(datas);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "matching_points_image");

    bounding_pub pu;

    ros::spin();

    return 0;
}
