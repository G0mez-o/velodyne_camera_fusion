
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

class bounding_pub{
    private:
        ros::NodeHandle nh;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2, darknet_ros_msgs::BoundingBoxes> fusion_bounding_subs;
        message_filters::Subscriber<sensor_msgs::Image> image_sub;
        message_filters::Subscriber<sensor_msgs::PointCloud2> proj_point_sub;
        message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub;
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_sub;
        message_filters::Synchronizer<fusion_bounding_subs> fusion_bounding_sync;
        ros::Publisher pub;
        ros::Publisher pub_;
    public:
  //コンストラクタ
        bounding_pub();
        void Callback(const sensor_msgs::Image::ConstPtr&, const sensor_msgs::CameraInfo::ConstPtr&, const sensor_msgs::PointCloud2::ConstPtr&, const darknet_ros_msgs::BoundingBoxes::ConstPtr&);
};

bounding_pub::bounding_pub()
    : nh("~"),
      image_sub(nh, "/occam/image0", 100), proj_point_sub(nh, "/fusion_points", 100), cam_info_sub(nh, "/camera_info", 100), bounding_sub(nh, "/darknet_ros/bounding_boxes", 100),
      fusion_bounding_sync(fusion_bounding_subs(10), image_sub, cam_info_sub, proj_point_sub, bounding_sub)

{
    printf("initialized!!\n");
    pub = nh.advertise<sensor_msgs::PointCloud2>("/object_points", 10);
    pub_ = nh.advertise<sensor_msgs::Image>("detected_person_image", 10);
    fusion_bounding_sync.registerCallback(boost::bind(&bounding_pub::Callback, this, _1, _2, _3, _4));
}

void bounding_pub::Callback(const sensor_msgs::Image::ConstPtr& image, const sensor_msgs::CameraInfo::ConstPtr& cinfo, const sensor_msgs::PointCloud2::ConstPtr& points, const darknet_ros_msgs::BoundingBoxes::ConstPtr& boxes)
{
  unsigned int object_num = boxes->bounding_boxes.size();
  printf("callback!\n");
  std::cout<<"cv_bridge"<<std::endl;
  cv_bridge::CvImageConstPtr cv_img_ptr;
  try{
    cv_img_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_8UC3);
  }catch (cv_bridge::Exception& e){
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
  cv::Mat cv_image(cv_img_ptr->image.rows, cv_img_ptr->image.cols, cv_img_ptr->image.type());
  cv_image = cv_img_ptr->image;
  cv::Mat rgb_image;
  cv::cvtColor(cv_image ,rgb_image, CV_BGR2RGB);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::fromROSMsg(*points, *cloud);
  image_geometry::PinholeCameraModel cam_model;
  cam_model.fromCameraInfo(cinfo);
  pcl::PointCloud<pcl::PointXYZRGB> msg;
  double range;
  for (unsigned int i = 0; i < object_num; i++)
  {
    bool init_flag = false;
    if (boxes->bounding_boxes[i].Class != "person" && boxes->bounding_boxes[i].probability < 35.0)
    {
      continue;
    }
    double mid_x, mid_y;
    double set_x, set_y;
    mid_x = (boxes->bounding_boxes[i].xmin + boxes->bounding_boxes[i].xmax) / 2.0;
    mid_y = (boxes->bounding_boxes[i].ymax + boxes->bounding_boxes[i].ymax) / 2.0;
    for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pt=cloud->points.begin(); pt<cloud->points.end(); pt++)
    {
      cv::Point3d p_cv((*pt).x, (*pt).y, (*pt).z);
      cv::Point2d pv;
      pv = cam_model.project3dToPixel(p_cv);
      cv::Point2d pv_(pv.x + 240, pv.y + 360);
      if (pv_.x > boxes->bounding_boxes[i].xmin && pv_.x < boxes->bounding_boxes[i].xmax && pv_.y > boxes->bounding_boxes[i].ymin && pv_.y < boxes->bounding_boxes[i].ymax)
      {
        if (!init_flag)
        {
          range = sqrt( pow((*pt).x, 2.0) + pow((*pt).z, 2.0));
          set_x = pv_.x;
          set_y = pv_.y;
        }
        else if (sqrt(pow(pv_.x - mid_x, 2.0) + pow(pv_.y - mid_y, 2.0)) < sqrt(pow(set_x - mid_x, 2.0) + pow(set_y - mid_y, 2.0)))
        {
          range = sqrt( pow((*pt).x, 2.0) + pow((*pt).z, 2.0));
          set_x = pv_.x;
          set_y = pv_.y;
        }
        pcl::PointXYZRGB buffer_point;
        buffer_point.x = (*pt).x;
        buffer_point.y = (*pt).y;
        buffer_point.z = (*pt).z;
        buffer_point.r = (*pt).r;
        buffer_point.g = (*pt).g;
        buffer_point.b = (*pt).b;
        msg.push_back(buffer_point);
      }
    }
    cv::rectangle(rgb_image, cv::Point(boxes->bounding_boxes[i].xmin, boxes->bounding_boxes[i].ymin), cv::Point(boxes->bounding_boxes[i].xmax,  boxes->bounding_boxes[i].ymax), cv::Scalar(255, 0, 255), 3, 4);
    std::string boxtext;
    boxtext += "probability:";
    boxtext += std::to_string(boxes->bounding_boxes[i].probability);
    boxtext += ", range:";
    boxtext += std::to_string(range);
    cv::putText(rgb_image, boxtext, cv::Point(boxes->bounding_boxes[i].xmin, boxes->bounding_boxes[i].ymin - 6), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(255, 0, 255), CV_AA);
  }
  auto detected_person_msg = msg.makeShared();
  detected_person_msg->header.frame_id = "/occam/image0";
  pcl_conversions::toPCL(ros::Time::now(), detected_person_msg->header.stamp);
  pub.publish(detected_person_msg);
  sensor_msgs::ImagePtr detected_image;
  detected_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb_image).toImageMsg();
  detected_image->header.frame_id = image->header.frame_id;
  detected_image->header.stamp = ros::Time::now();
  pub_.publish(detected_image);
  printf("finished measuring objects!!\n");
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "matching_points_image");

    bounding_pub pu;

    ros::spin();

    return 0;
}
