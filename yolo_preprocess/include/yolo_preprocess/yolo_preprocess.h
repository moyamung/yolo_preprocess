#include <ros/ros.h>
#include <ros/package.h>
#include <sstream>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include "final_result_msgs/save_image.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"

class Yolo_preprocess
{
    public:
      Yolo_preprocess();
      ~Yolo_preprocess();

      double target_x, target_y;
      int target_class_id;
      std::vector<int> detected_num_list; 


    private:
      cv::Mat now_Image;
      cv::Mat best_Images[10];

      float confidence[10];

      ros::NodeHandle node_handle_;

      ros::Subscriber target_detection_img;
      ros::Subscriber yolo_detection_image;

      ros::Publisher final_result_pub;

      void initPublisher();
      void initSubscriber();
      void ImageDetectionCallback(const sensor_msgs::ImageConstPtr& msg);
      void YoloDetectionCallBack(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg);

      void SendImage(int target_class, cv::Mat image);

      const float CalculateConfidence(int x_size, int y_size, float probability);
};
