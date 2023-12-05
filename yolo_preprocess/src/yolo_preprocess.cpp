#include "yolo_preprocess/yolo_preprocess.h"

Yolo_preprocess::Yolo_preprocess()
: node_handle_("")
{
    initPublisher();
    initSubscriber();

    for (int i = 0; i < 10; i++)
    {
        confidence[i] = 0;
    }

    ROS_INFO("Img preprocessor ready");
}

Yolo_preprocess::~Yolo_preprocess()
{
    ros::Rate loop_rate(10);

    ROS_INFO("save others");
    for (int i = 0; i < 10; i++)
    {
        if (confidence[i] < 0.001)
            continue;
        ROS_INFO("sending %d", i);
        SendImage(i, best_Images[i]);
        loop_rate.sleep();
    }
    ros::shutdown();
}

void Yolo_preprocess::initPublisher()
{
    final_result_pub = node_handle_.advertise<final_result_msgs::save_image>("/target_detection", 1000);
}

void Yolo_preprocess::initSubscriber()
{
    target_detection_img = node_handle_.subscribe("/camera/color/image_raw", 1, &Yolo_preprocess::ImageDetectionCallback, this);
    yolo_detection_image = node_handle_.subscribe("/darknet_ros/bounding_boxes", 1, &Yolo_preprocess::YoloDetectionCallBack, this);
}

void Yolo_preprocess::ImageDetectionCallback(const sensor_msgs::ImageConstPtr& msg)
{
    cv::Mat depth_pic;
    cv_bridge::CvImagePtr depth_ptr;

    try
    {   
        depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); 
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'mono16'.", msg->encoding.c_str());
    }
    now_Image = depth_ptr->image;
}

void Yolo_preprocess::YoloDetectionCallBack(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
    //printf("%d %d %d %d\n", msg->bounding_boxes[0].xmin, msg->bounding_boxes[0].ymin, msg->bounding_boxes[0].xmax, msg->bounding_boxes[0].ymax);
    int xmin = msg->bounding_boxes[0].xmin;
    int ymin = msg->bounding_boxes[0].ymin;
    int xmax = msg->bounding_boxes[0].xmax;
    int ymax = msg->bounding_boxes[0].ymax;
    cv::Mat cropped_image = now_Image.clone()(cv::Range(ymin, ymax), cv::Range(xmin,xmax));
    float now_conf = CalculateConfidence(xmax - xmin, ymax - ymin, msg->bounding_boxes[0].probability);

    std::cout << now_conf << std::endl;

    std::string target_class_id = msg->bounding_boxes[0].Class;

    int target_class = msg->bounding_boxes[0].id;

    if (now_conf > confidence[target_class])
    {
        confidence[target_class] = now_conf;
        best_Images[target_class] = cropped_image;
        std::stringstream filename;
        filename << "/home/whjeong/yolo_crop/" << target_class_id << ".jpg";
                
        std::cout << filename.str() << std::endl;

        cv::imwrite(filename.str(), cropped_image);

        if (now_conf > 0.06)
        {
            SendImage(target_class, cropped_image);
        }
    }
}

void Yolo_preproocess::SendImage(int target_class, cv::Mat image)
{
    final_result_msgs::save_image send_msg;
    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_image).toImageMsg();
    send_msg.class_id = target_class;
    send_msg.save_img = *img_msg;
    send_msg.x_pose = 0;
    send_msg.y_pose = 0;
    
    final_result_pub.publish(send_msg);
}

const float Yolo_preprocess::CalculateConfidence(int x_size, int y_size, float probability)
{
    if (probability < 0.65)
        return 0;
    float x_prop = (float)x_size / 640.0;
    float y_prop = (float)y_size / 480.0;
    return x_prop * y_prop * probability * probability;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "yolo_preprocess");
    Yolo_preprocess yolo_preprocess;
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
